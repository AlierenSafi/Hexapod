package com.example.hexapodtester

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import android.os.Handler
import android.os.Looper
import android.util.Log
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import org.json.JSONObject
import java.util.*

enum class ConnectionState {
    DISCONNECTED, CONNECTING, CONNECTED
}

enum class TestStatus {
    PENDING, PASSED, FAILED, SKIPPED
}

data class TestResults(
    val connectionStatus: TestStatus = TestStatus.PENDING,
    val commandTxStatus: TestStatus = TestStatus.PENDING,
    val telemetryRxStatus: TestStatus = TestStatus.PENDING,
    val batteryStatus: TestStatus = TestStatus.PENDING,
    val imuStatus: TestStatus = TestStatus.PENDING,
    val pcaStatus: TestStatus = TestStatus.PENDING
) {
    val allPassed: Boolean
        get() = (connectionStatus == TestStatus.PASSED || connectionStatus == TestStatus.SKIPPED) &&
                (commandTxStatus == TestStatus.PASSED || commandTxStatus == TestStatus.SKIPPED) &&
                (telemetryRxStatus == TestStatus.PASSED || telemetryRxStatus == TestStatus.SKIPPED) &&
                (batteryStatus == TestStatus.PASSED || batteryStatus == TestStatus.SKIPPED) &&
                (imuStatus == TestStatus.PASSED || imuStatus == TestStatus.SKIPPED) &&
                (pcaStatus == TestStatus.PASSED || pcaStatus == TestStatus.SKIPPED)
}

@SuppressLint("MissingPermission")
class BleManager(private val context: Context) {

    private val TAG = "BleManager"

    // UUIDs matching communication.h
    private val SERVICE_UUID = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
    private val CHAR_RX_UUID = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E") // Write
    private val CHAR_TX_UUID = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E") // Notify
    private val CLIENT_CONFIG_DESCRIPTOR_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    private val bluetoothAdapter: BluetoothAdapter? by lazy {
        val manager = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        manager.adapter
    }

    private var bluetoothGatt: BluetoothGatt? = null
    private var rxCharacteristic: BluetoothGattCharacteristic? = null
    private var txCharacteristic: BluetoothGattCharacteristic? = null

    private val _isScanning = MutableStateFlow(false)
    val isScanning: StateFlow<Boolean> = _isScanning.asStateFlow()

    private val _discoveredDevices = MutableStateFlow<List<BluetoothDevice>>(emptyList())
    val discoveredDevices: StateFlow<List<BluetoothDevice>> = _discoveredDevices.asStateFlow()

    private val _connectionState = MutableStateFlow(ConnectionState.DISCONNECTED)
    val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    private val _telemetryJson = MutableStateFlow<String?>(null)
    val telemetryJson: StateFlow<String?> = _telemetryJson.asStateFlow()

    private val _logs = MutableStateFlow<List<String>>(emptyList())
    val logs: StateFlow<List<String>> = _logs.asStateFlow()

    private val _testResults = MutableStateFlow(TestResults())
    val testResults: StateFlow<TestResults> = _testResults.asStateFlow()

    private val handler = Handler(Looper.getMainLooper())

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult?) {
            result?.device?.let { device ->
                val current = _discoveredDevices.value
                if (current.none { it.address == device.address }) {
                    _discoveredDevices.value = current + device
                }
            }
        }

        override fun onScanFailed(errorCode: Int) {
            log("Scan failed with error code: $errorCode")
            _isScanning.value = false
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    _connectionState.value = ConnectionState.CONNECTED
                    log("Connected to ${gatt?.device?.name ?: "Unknown"}. Requesting MTU 512...")
                    _testResults.value = _testResults.value.copy(connectionStatus = TestStatus.PASSED)
                    handler.post {
                        gatt?.requestMtu(512)
                    }
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    handleDisconnect()
                }
            } else {
                log("GATT error: status $status")
                handleDisconnect()
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt?, mtu: Int, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                log("MTU changed to $mtu. Discovering services...")
            } else {
                log("MTU request failed: status $status. Discovering services with default MTU...")
            }
            handler.post {
                gatt?.discoverServices()
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt?, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                val service = gatt?.getService(SERVICE_UUID)
                if (service != null) {
                    rxCharacteristic = service.getCharacteristic(CHAR_RX_UUID)
                    txCharacteristic = service.getCharacteristic(CHAR_TX_UUID)

                    if (txCharacteristic != null) {
                        gatt.setCharacteristicNotification(txCharacteristic, true)
                        val descriptor = txCharacteristic?.getDescriptor(CLIENT_CONFIG_DESCRIPTOR_UUID)
                        if (descriptor != null) {
                            descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                            gatt.writeDescriptor(descriptor)
                            log("Subscribed to Telemetry notifications.")
                        }
                    } else {
                        log("Telemetry TX characteristic not found!")
                    }
                } else {
                    log("Hexapod service not found on device!")
                }
            } else {
                log("Service discovery failed: status $status")
            }
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?) {
            if (characteristic?.uuid == CHAR_TX_UUID) {
                val data = characteristic.getStringValue(0) ?: ""
                _telemetryJson.value = data
                log("Telemetry RX (Legacy): $data")
                parseTelemetry(data)
            }
        }

        override fun onCharacteristicChanged(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic, value: ByteArray) {
            if (characteristic.uuid == CHAR_TX_UUID) {
                val data = String(value, Charsets.UTF_8)
                _telemetryJson.value = data
                log("Telemetry RX: $data")
                parseTelemetry(data)
            }
        }

        override fun onCharacteristicWrite(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                log("Command TX Success")
                _testResults.value = _testResults.value.copy(commandTxStatus = TestStatus.PASSED)
            } else {
                log("Command TX Failed: status $status")
            }
        }
    }

    fun startScan() {
        if (bluetoothAdapter == null) return
        _discoveredDevices.value = emptyList()
        _isScanning.value = true
        log("Starting BLE scan...")
        bluetoothAdapter?.bluetoothLeScanner?.startScan(scanCallback)

        // Stop scan after 10 seconds automatically
        handler.postDelayed({
            if (_isScanning.value) stopScan()
        }, 10000)
    }

    fun stopScan() {
        if (bluetoothAdapter == null) return
        log("Stopping BLE scan.")
        bluetoothAdapter?.bluetoothLeScanner?.stopScan(scanCallback)
        _isScanning.value = false
    }

    fun connect(device: BluetoothDevice) {
        stopScan()
        _connectionState.value = ConnectionState.CONNECTING
        log("Connecting to ${device.name ?: device.address}...")
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
    }

    fun disconnect() {
        log("Disconnecting...")
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        handleDisconnect()
    }

    private fun handleDisconnect() {
        _connectionState.value = ConnectionState.DISCONNECTED
        rxCharacteristic = null
        txCharacteristic = null
        bluetoothGatt = null
        _testResults.value = TestResults() // reset checks
        log("Disconnected.")
    }

    fun sendCommand(json: String) {
        val char = rxCharacteristic
        val gatt = bluetoothGatt
        if (gatt == null || char == null) {
            log("Cannot send: not connected or RX characteristic not found.")
            return
        }
        log("Command TX: $json")
        char.value = json.toByteArray()
        gatt.writeCharacteristic(char)
    }

    fun runAutoTest() {
        if (_connectionState.value != ConnectionState.CONNECTED) {
            log("Auto test: Not connected.")
            return
        }
        log("Auto test started.")
        // Reset results to PENDING (except connection which is PASSED)
        _testResults.value = TestResults(
            connectionStatus = TestStatus.PASSED,
            commandTxStatus = TestStatus.PENDING,
            telemetryRxStatus = TestStatus.PENDING,
            batteryStatus = TestStatus.PENDING,
            imuStatus = TestStatus.PENDING,
            pcaStatus = TestStatus.PENDING
        )
        
        // 1. Trigger Command Write Test by sending a stop command
        sendCommand("{\"cmd\":\"motion\",\"x\":0,\"y\":0,\"yaw\":0}")
        
        // 2. Schedule a timeout to evaluate pending tests based on hardware presence
        handler.postDelayed({
            val current = _testResults.value
            var updated = current
            
            // If telemetry was never received, we can't detect hardware. Mark all pending as FAILED.
            if (current.telemetryRxStatus == TestStatus.PENDING) {
                updated = updated.copy(
                    telemetryRxStatus = TestStatus.FAILED,
                    batteryStatus = TestStatus.FAILED,
                    imuStatus = TestStatus.FAILED,
                    pcaStatus = TestStatus.FAILED
                )
            } else {
                // If telemetry was received, evaluate pending statuses based on detected hardware
                if (updated.imuStatus == TestStatus.PENDING) {
                    updated = updated.copy(imuStatus = TestStatus.FAILED)
                }
                if (updated.batteryStatus == TestStatus.PENDING) {
                    updated = updated.copy(batteryStatus = TestStatus.FAILED)
                }
                if (updated.pcaStatus == TestStatus.PENDING) {
                    updated = updated.copy(pcaStatus = TestStatus.FAILED)
                }
            }
            
            if (updated.commandTxStatus == TestStatus.PENDING) {
                updated = updated.copy(commandTxStatus = TestStatus.FAILED)
            }
            
            _testResults.value = updated
            log("Auto test completed. All passed: ${updated.allPassed}")
        }, 4000) // Wait 4 seconds to receive slow + fast telemetry packets
    }

    private fun parseTelemetry(jsonStr: String) {
        try {
            val json = JSONObject(jsonStr)
            _testResults.value = _testResults.value.copy(telemetryRxStatus = TestStatus.PASSED)

            val type = json.optString("t")
            if (type == "fast") {
                val imu = json.optJSONObject("imu")
                if (imu != null && _testResults.value.imuStatus != TestStatus.SKIPPED) {
                    val pitch = imu.optDouble("p", 0.0)
                    val roll = imu.optDouble("r", 0.0)
                    // If we receive active (non-zero or changing) orientation, pass IMU
                    if (pitch != 0.0 || roll != 0.0) {
                        _testResults.value = _testResults.value.copy(imuStatus = TestStatus.PASSED)
                    }
                }
            } else if (type == "slow") {
                // Read hardware availability
                val hw = json.optJSONObject("hw")
                if (hw != null) {
                    val imuOnline = hw.optInt("imu", 0) == 1
                    val pcaOnline = hw.optInt("pca", 0) == 1
                    
                    if (!imuOnline) {
                        _testResults.value = _testResults.value.copy(imuStatus = TestStatus.SKIPPED)
                    }
                    if (!pcaOnline) {
                        _testResults.value = _testResults.value.copy(pcaStatus = TestStatus.SKIPPED)
                    } else if (_testResults.value.pcaStatus == TestStatus.PENDING) {
                        _testResults.value = _testResults.value.copy(pcaStatus = TestStatus.PASSED)
                    }
                } else {
                    // Fallback to skipped if older firmware doesn't report hardware block
                    _testResults.value = _testResults.value.copy(
                        imuStatus = TestStatus.SKIPPED,
                        pcaStatus = TestStatus.SKIPPED
                    )
                }

                val batt = json.optJSONObject("batt")
                if (batt != null) {
                    val voltage = batt.optDouble("v", 0.0)
                    if (voltage < 1.0) {
                        _testResults.value = _testResults.value.copy(batteryStatus = TestStatus.SKIPPED)
                    } else if (voltage > 6.4) {
                        _testResults.value = _testResults.value.copy(batteryStatus = TestStatus.PASSED)
                    } else {
                        _testResults.value = _testResults.value.copy(batteryStatus = TestStatus.FAILED)
                    }
                }
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error parsing telemetry JSON", e)
        }
    }

    private fun log(message: String) {
        val time = java.text.SimpleDateFormat("HH:mm:ss.SSS", java.util.Locale.getDefault()).format(Date())
        val line = "[$time] $message"
        _logs.value = listOf(line) + _logs.value.take(99) // limit logs to 100 lines
        Log.d(TAG, message)
    }
}

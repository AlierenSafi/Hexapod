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

data class TestResults(
    val connectionPassed: Boolean = false,
    val commandTxPassed: Boolean = false,
    val telemetryRxPassed: Boolean = false,
    val batteryPassed: Boolean = false,
    val imuPassed: Boolean = false
) {
    val allPassed: Boolean
        get() = connectionPassed && commandTxPassed && telemetryRxPassed && batteryPassed && imuPassed
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
                    log("Connected to ${gatt?.device?.name ?: "Unknown"}")
                    _testResults.value = _testResults.value.copy(connectionPassed = true)
                    handler.post {
                        gatt?.discoverServices()
                    }
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    handleDisconnect()
                }
            } else {
                log("GATT error: status $status")
                handleDisconnect()
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

        override fun onCharacteristicChanged(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?) {
            if (characteristic?.uuid == CHAR_TX_UUID) {
                val data = characteristic.getStringValue(0) ?: ""
                _telemetryJson.value = data
                log("Telemetry RX: $data")
                parseTelemetry(data)
            }
        }

        override fun onCharacteristicWrite(gatt: BluetoothGatt?, characteristic: BluetoothGattCharacteristic?, status: Int) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                log("Command TX Success")
                _testResults.value = _testResults.value.copy(commandTxPassed = true)
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

    private fun parseTelemetry(jsonStr: String) {
        try {
            val json = JSONObject(jsonStr)
            _testResults.value = _testResults.value.copy(telemetryRxPassed = true)

            val type = json.optString("t")
            if (type == "fast") {
                val imu = json.optJSONObject("imu")
                if (imu != null) {
                    val pitch = imu.optDouble("p", 0.0)
                    val roll = imu.optDouble("r", 0.0)
                    // If we receive active (non-zero or changing) orientation, pass IMU
                    if (pitch != 0.0 || roll != 0.0) {
                        _testResults.value = _testResults.value.copy(imuPassed = true)
                    }
                }
            } else if (type == "slow") {
                val batt = json.optJSONObject("batt")
                if (batt != null) {
                    val voltage = batt.optDouble("v", 0.0)
                    if (voltage > 6.4) {
                        _testResults.value = _testResults.value.copy(batteryPassed = true)
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

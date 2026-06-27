package com.example.hexapodtester.ui.main

import android.annotation.SuppressLint
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.hexapodtester.BleManager
import com.example.hexapodtester.ConnectionState
import com.example.hexapodtester.TestResults
import com.example.hexapodtester.TestStatus

@SuppressLint("MissingPermission")
@Composable
fun MainScreen(
    bleManager: BleManager,
    modifier: Modifier = Modifier
) {
    val isScanning by bleManager.isScanning.collectAsState()
    val devices by bleManager.discoveredDevices.collectAsState()
    val connState by bleManager.connectionState.collectAsState()
    val telemetry by bleManager.telemetryJson.collectAsState()
    val logs by bleManager.logs.collectAsState()
    val tests by bleManager.testResults.collectAsState()

    var showDevicesDialog by remember { mutableStateOf(false) }

    Column(
        modifier = modifier
            .fillMaxSize()
            .background(Color(0xFF121212)) // Pure Dark Background
            .padding(16.dp)
            .verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        // App Header
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column {
                Text(
                    text = "Hexapod Diagnostics",
                    color = Color.White,
                    fontSize = 24.sp,
                    fontWeight = FontWeight.Bold
                )
                Text(
                    text = "Dual-Core BLE Test System",
                    color = Color.Gray,
                    fontSize = 12.sp
                )
            }

            // Connection Status Badge
            val badgeColor = when (connState) {
                ConnectionState.CONNECTED -> Color(0xFF4CAF50)
                ConnectionState.CONNECTING -> Color(0xFFFFC107)
                ConnectionState.DISCONNECTED -> Color(0xFFF44336)
            }
            val badgeText = when (connState) {
                ConnectionState.CONNECTED -> "CONNECTED"
                ConnectionState.CONNECTING -> "CONNECTING..."
                ConnectionState.DISCONNECTED -> "DISCONNECTED"
            }

            Card(
                colors = CardDefaults.cardColors(containerColor = badgeColor.copy(alpha = 0.15f)),
                shape = RoundedCornerShape(8.dp)
            ) {
                Text(
                    text = badgeText,
                    color = badgeColor,
                    fontSize = 12.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(horizontal = 12.dp, vertical = 6.dp)
                )
            }
        }

        // Connection Action Panel
        Card(
            colors = CardDefaults.cardColors(containerColor = Color(0xFF1E1E1E)),
            shape = RoundedCornerShape(12.dp)
        ) {
            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(16.dp),
                horizontalArrangement = Arrangement.spacedBy(12.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                if (connState == ConnectionState.DISCONNECTED) {
                    Button(
                        onClick = {
                            bleManager.startScan()
                            showDevicesDialog = true
                        },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2196F3)),
                        modifier = Modifier.weight(1f)
                    ) {
                        Text("Scan & Connect", color = Color.White)
                    }
                } else {
                    Button(
                        onClick = { bleManager.runAutoTest() },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF4CAF50)),
                        modifier = Modifier.weight(1f)
                    ) {
                        Text("Auto Test", color = Color.White)
                    }
                    Button(
                        onClick = { bleManager.disconnect() },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFE91E63)),
                        modifier = Modifier.weight(1f)
                    ) {
                        Text("Disconnect", color = Color.White)
                    }
                }
            }
        }

        // Test Status Panel
        TestStatusPanel(tests)

        // Motion Controller Card
        if (connState == ConnectionState.CONNECTED) {
            MotionControllerPanel(onSendCommand = { bleManager.sendCommand(it) })
            ParameterTuningPanel(onSendCommand = { bleManager.sendCommand(it) })
            TelemetryMonitorPanel(telemetry)
        }

        // Logs/Terminal Card
        LogsPanel(logs)
    }

    // Devices Dialog
    if (showDevicesDialog) {
        AlertDialog(
            onDismissRequest = {
                bleManager.stopScan()
                showDevicesDialog = false
            },
            title = { Text("Discovered Devices", color = Color.White) },
            text = {
                Box(
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(300.dp)
                ) {
                    if (devices.isEmpty() && isScanning) {
                        CircularProgressIndicator(
                            color = Color.Blue,
                            modifier = Modifier.align(Alignment.Center)
                        )
                    } else if (devices.isEmpty()) {
                        Text(
                            "No devices found. Ensure ESP32 is powered.",
                            color = Color.Gray,
                            modifier = Modifier.align(Alignment.Center)
                        )
                    } else {
                        LazyColumn(
                            verticalArrangement = Arrangement.spacedBy(8.dp)
                        ) {
                            items(devices) { device ->
                                Card(
                                    colors = CardDefaults.cardColors(containerColor = Color(0xFF2C2C2C)),
                                    modifier = Modifier
                                        .fillMaxWidth()
                                        .clickable {
                                            bleManager.connect(device)
                                            showDevicesDialog = false
                                        }
                                ) {
                                    Column(modifier = Modifier.padding(12.dp)) {
                                        Text(device.name ?: "Unknown Device", color = Color.White, fontWeight = FontWeight.Bold)
                                        Text(device.address, color = Color.Gray, fontSize = 12.sp)
                                    }
                                }
                            }
                        }
                    }
                }
            },
            confirmButton = {
                TextButton(
                    onClick = {
                        bleManager.stopScan()
                        showDevicesDialog = false
                    }
                ) {
                    Text("Close", color = Color.White)
                }
            },
            containerColor = Color(0xFF1E1E1E)
        )
    }
}

@Composable
fun TestStatusPanel(results: TestResults) {
    val allPassed = results.allPassed
    val containerColor = if (allPassed) Color(0xFF1B5E20) else Color(0xFF2C2C2C)

    Card(
        colors = CardDefaults.cardColors(containerColor = containerColor),
        shape = RoundedCornerShape(12.dp),
        border = CardDefaults.outlinedCardBorder().copy(width = 1.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Text(
                    text = if (allPassed) "✅" else "⚠️",
                    fontSize = 20.sp
                )
                Text(
                    text = if (allPassed) "ALL DETECTED SYSTEM TESTS PASSED" else "SYSTEM TESTING IN PROGRESS",
                    color = Color.White,
                    fontWeight = FontWeight.Bold,
                    fontSize = 16.sp
                )
            }

            Divider(color = Color.White.copy(alpha = 0.1f))

            TestCheckItem(label = "Connection Established", status = results.connectionStatus)
            TestCheckItem(label = "Command Write (TX) Test", status = results.commandTxStatus)
            TestCheckItem(label = "Telemetry Packet (RX) Test", status = results.telemetryRxStatus)
            TestCheckItem(label = "PCA9685 Servo Driver Connection", status = results.pcaStatus)
            TestCheckItem(label = "IMU Leveling Alignment", status = results.imuStatus)
            TestCheckItem(label = "Battery Safety Threshold (> 6.4V)", status = results.batteryStatus)
        }
    }
}

@Composable
fun TestCheckItem(label: String, status: TestStatus) {
    val statusText = when (status) {
        TestStatus.PASSED -> "PASSED"
        TestStatus.PENDING -> "PENDING"
        TestStatus.FAILED -> "FAILED"
        TestStatus.SKIPPED -> "SKIPPED (N/A)"
    }
    val statusColor = when (status) {
        TestStatus.PASSED -> Color(0xFF4CAF50)
        TestStatus.PENDING -> Color(0xFFFFB74D)
        TestStatus.FAILED -> Color(0xFFF44336)
        TestStatus.SKIPPED -> Color(0xFF90A4AE)
    }

    Row(
        modifier = Modifier.fillMaxWidth(),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(text = label, color = Color.LightGray, fontSize = 14.sp)
        Text(
            text = statusText,
            color = statusColor,
            fontSize = 12.sp,
            fontWeight = FontWeight.Bold
        )
    }
}

@Composable
fun MotionControllerPanel(onSendCommand: (String) -> Unit) {
    Card(
        colors = CardDefaults.cardColors(containerColor = Color(0xFF1E1E1E)),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Text("Motion Testing Control", color = Color.White, fontWeight = FontWeight.Bold)

            // D-Pad Grid
            Column(
                modifier = Modifier.fillMaxWidth(),
                horizontalAlignment = Alignment.CenterHorizontally,
                verticalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Button(
                    onClick = { onSendCommand("{\"cmd\":\"motion\",\"x\":50,\"y\":0,\"yaw\":0}") },
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF333333))
                ) {
                    Text("FORWARD", color = Color.White)
                }

                Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Button(
                        onClick = { onSendCommand("{\"cmd\":\"motion\",\"x\":0,\"y\":-50,\"yaw\":0}") },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF333333))
                    ) {
                        Text("LEFT", color = Color.White)
                    }
                    Button(
                        onClick = { onSendCommand("{\"cmd\":\"motion\",\"x\":0,\"y\":0,\"yaw\":0}") },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFE91E63))
                    ) {
                        Text("STOP", color = Color.White)
                    }
                    Button(
                        onClick = { onSendCommand("{\"cmd\":\"motion\",\"x\":0,\"y\":50,\"yaw\":0}") },
                        colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF333333))
                    ) {
                        Text("RIGHT", color = Color.White)
                    }
                }

                Button(
                    onClick = { onSendCommand("{\"cmd\":\"motion\",\"x\":-50,\"y\":0,\"yaw\":0}") },
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF333333))
                ) {
                    Text("BACKWARD", color = Color.White)
                }
            }

            Divider(color = Color.White.copy(alpha = 0.1f))

            // Extra functions
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(12.dp)
            ) {
                Button(
                    onClick = { onSendCommand("{\"cmd\":\"system\",\"action\":\"stand_up\"}") },
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2C2C2C)),
                    modifier = Modifier.weight(1f)
                ) {
                    Text("STAND UP", color = Color.White)
                }
                Button(
                    onClick = { onSendCommand("{\"cmd\":\"system\",\"action\":\"sit_down\"}") },
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2C2C2C)),
                    modifier = Modifier.weight(1f)
                ) {
                    Text("SIT DOWN", color = Color.White)
                }
            }
        }
    }
}

@Composable
fun ParameterTuningPanel(onSendCommand: (String) -> Unit) {
    var stepHeight by remember { mutableStateOf(30f) }
    var speed by remember { mutableStateOf(50f) }

    Card(
        colors = CardDefaults.cardColors(containerColor = Color(0xFF1E1E1E)),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(12.dp)
        ) {
            Text("Parameter Testing", color = Color.White, fontWeight = FontWeight.Bold)

            Text("Step Height: ${stepHeight.toInt()} mm", color = Color.LightGray, fontSize = 14.sp)
            Slider(
                value = stepHeight,
                onValueChange = { stepHeight = it },
                valueRange = 10f..50f,
                colors = SliderDefaults.colors(thumbColor = Color(0xFF2196F3), activeTrackColor = Color(0xFF2196F3))
            )

            Text("Speed: ${speed.toInt()} %", color = Color.LightGray, fontSize = 14.sp)
            Slider(
                value = speed,
                onValueChange = { speed = it },
                valueRange = 10f..100f,
                colors = SliderDefaults.colors(thumbColor = Color(0xFF2196F3), activeTrackColor = Color(0xFF2196F3))
            )

            Button(
                onClick = {
                    onSendCommand("{\"cmd\":\"param.set\",\"key\":\"stepHeight\",\"value\":${stepHeight.toInt()},\"save\":true}")
                    onSendCommand("{\"cmd\":\"param.set\",\"key\":\"gaitSpeed\",\"value\":${speed.toInt()},\"save\":true}")
                },
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFF2196F3)),
                modifier = Modifier.fillMaxWidth()
            ) {
                Text("Sync & Save parameters", color = Color.White)
            }
        }
    }
}

@Composable
fun TelemetryMonitorPanel(json: String?) {
    Card(
        colors = CardDefaults.cardColors(containerColor = Color(0xFF1E1E1E)),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("Real-Time Telemetry Monitor", color = Color.White, fontWeight = FontWeight.Bold)
            Divider(color = Color.White.copy(alpha = 0.1f))

            if (json == null) {
                Text("Awaiting first telemetry package...", color = Color.Gray, fontSize = 12.sp)
            } else {
                Text(
                    text = json,
                    color = Color(0xFF81C784),
                    fontSize = 12.sp,
                    fontFamily = FontFamily.Monospace,
                    modifier = Modifier
                        .fillMaxWidth()
                        .background(Color.Black.copy(alpha = 0.2f))
                        .padding(8.dp)
                )
            }
        }
    }
}

@Composable
fun LogsPanel(logs: List<String>) {
    Card(
        colors = CardDefaults.cardColors(containerColor = Color(0xFF1E1E1E)),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("Communication Console Log", color = Color.White, fontWeight = FontWeight.Bold)
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(150.dp)
                    .background(Color.Black, shape = RoundedCornerShape(6.dp))
                    .padding(8.dp)
            ) {
                LazyColumn(modifier = Modifier.fillMaxSize()) {
                    items(logs) { logLine ->
                        Text(
                            text = logLine,
                            color = Color(0xFF00FF00),
                            fontSize = 11.sp,
                            fontFamily = FontFamily.Monospace
                        )
                    }
                }
            }
        }
    }
}

package com.example.whi_livepole

import android.bluetooth.BluetoothDevice
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.foundation.Image
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.layout.ContentScale

@Composable
fun BluetoothScreen(
    scanDevices: () -> List<BluetoothDevice>,
    connectToDevice: (BluetoothDevice) -> Unit,
    sendCommand: (String) -> Unit,
    getConnectedDevice: () -> String?,
    incomingMessages: List<String>,
) {
    var devices by remember { mutableStateOf<List<BluetoothDevice>>(emptyList()) }
    var showDevices by remember { mutableStateOf(true) }
    val connectedDevice = getConnectedDevice()
    var showSettingsDialog by remember { mutableStateOf(false) }
    var showNotConnectedPopup by remember { mutableStateOf(false) }
    val coroutineScope = rememberCoroutineScope()

    // ---- Compass/Magnetometer state (auto-färg) ----
    var magnetometerEnabled by remember { mutableStateOf<Boolean?>(null) }
    LaunchedEffect(incomingMessages) {
        incomingMessages.lastOrNull()?.let { msg ->
            if (msg.contains("X1")) magnetometerEnabled = true
            if (msg.contains("X0")) magnetometerEnabled = false
        }
    }

    // === För MANUAL COMMAND: Dessa måste ligga här för att dela state mellan dialog och settings ===
    var showManualCommandDialog by remember { mutableStateOf(false) }
    var lastManualCommand by remember { mutableStateOf("") }
    var manualCommandResult by remember { mutableStateOf("") }
    var recentCommands by remember { mutableStateOf<List<String>>(emptyList()) }
    var showFeedbackDialog by remember { mutableStateOf(false) }

    val isConnected = connectedDevice != null

    // Visa feedback när ett manuellt kommando fått svar!
    LaunchedEffect(incomingMessages, lastManualCommand) {
        if (lastManualCommand.isNotBlank() && incomingMessages.isNotEmpty()) {
            val lastMsg = incomingMessages.last()
            if (lastMsg.isNotBlank()) {
                manualCommandResult = lastMsg
                showFeedbackDialog = true
            }
        }
    }

    Box(
        modifier = Modifier.fillMaxSize()
    ) {
        // Bakgrundsbild
        Image(
            painter = painterResource(id = R.drawable.background),
            contentDescription = null,
            modifier = Modifier.fillMaxSize(),
            contentScale = ContentScale.Crop
        )
        // Vit overlay
        Box(
            Modifier
                .fillMaxSize()
                .background(Color.White.copy(alpha = 0.25f))
        )
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(top = 60.dp, start = 20.dp, end = 20.dp, bottom = 20.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            // Inställningshjul
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.End
            ) {
                IconButton(onClick = { showSettingsDialog = true }) {
                    Icon(Icons.Default.Settings, contentDescription = "Settings")
                }
            }
            Text(
                "WLI SONAR POLE",
                style = MaterialTheme.typography.headlineSmall.copy(
                    fontSize = 30.sp,
                    fontWeight = FontWeight.Bold
                ),
                modifier = Modifier.fillMaxWidth(),
                textAlign = TextAlign.Center
            )
            Spacer(Modifier.height(30.dp))

            // Sök och lista enheter
            if (connectedDevice == null) {
                Button(
                    onClick = {
                        devices = scanDevices()
                        showDevices = true
                    },
                    modifier = Modifier.fillMaxWidth(0.7f)
                ) {
                    Text(
                        "Search Sonar Pole",
                        fontSize = 20.sp,
                        fontWeight = FontWeight.Bold
                    )
                }
                Spacer(Modifier.height(8.dp))
            }

            if (devices.isNotEmpty() && showDevices && connectedDevice == null) {
                Text(
                    "Connect:",
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
                LazyColumn(Modifier.height(200.dp)) {
                    items(devices.size) { idx ->
                        val dev = devices[idx]
                        Button(
                            onClick = {
                                connectToDevice(dev)
                                showDevices = false
                            },
                            Modifier
                                .fillMaxWidth()
                                .padding(2.dp)
                        ) {
                            Text(
                                "${dev.name} (${dev.address})",
                                fontSize = 18.sp,
                                fontWeight = FontWeight.Bold
                            )
                        }
                    }
                }
            }

            // Inställningsdialogen
            if (showSettingsDialog) {
                SettingsDialog(
                    onCommand = { cmd -> sendCommand(cmd) },
                    onSendValueCommand = { prefix, value ->
                        sendCommand(prefix + value)
                        sendCommand("O")
                    },
                    onDismiss = { showSettingsDialog = false },
                    magnetometerEnabled = magnetometerEnabled,
                    setMagnetometerEnabled = { enabled ->
                        magnetometerEnabled = enabled
                    },
                    lastMessage = incomingMessages.lastOrNull() ?: "",
                    isConnected = isConnected,
                    showManualCommandDialog = showManualCommandDialog,
                    onShowManualCommandDialog = { showManualCommandDialog = it },
                    recentCommands = recentCommands,
                    onAddManualCommand = { cmd ->
                        recentCommands = (listOf(cmd) + recentCommands).take(10)
                        lastManualCommand = cmd
                        manualCommandResult = "Waiting..."
                        sendCommand(cmd)
                    },
                    onClearCommands = { recentCommands = emptyList() },
                    manualCommandResult = manualCommandResult
                )
            }

            if (isConnected) {
                Text(
                    "Connected to: $connectedDevice",
                    color = Color(0xFF43A047),
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
                Spacer(Modifier.height(16.dp))
            }

            val textStyle = MaterialTheme.typography.titleMedium.copy(
                fontSize = 20.sp, fontWeight = FontWeight.Bold, color = Color(0xFF222222)
            )

            fun guardedSendCommand(cmd: String) {
                if (isConnected) {
                    sendCommand(cmd)
                } else {
                    showNotConnectedPopup = true
                }
            }

            Row(Modifier.fillMaxWidth(), Arrangement.SpaceEvenly) {
                MutedButton(
                    "Manual Speed +", "+", Color(0xFFB0BEC5), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("+") }
                MutedButton(
                    "Manual Speed -", "-", Color(0xFFB0BEC5), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("-") }
            }
            Spacer(Modifier.height(8.dp))

            Row(Modifier.fillMaxWidth(), Arrangement.SpaceEvenly) {
                MutedButton(
                    "Sweep Speed +", "U", Color(0xFFC8E6C9), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("U") }
                MutedButton(
                    "Sweep Speed -", "H", Color(0xFFC8E6C9), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("H") }
            }
            Spacer(Modifier.height(8.dp))

            Row(Modifier.fillMaxWidth(), Arrangement.SpaceEvenly) {
                MutedButton(
                    "Sweep Angle +", "N", Color(0xFFD7CCC8), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("N") }
                MutedButton(
                    "Sweep Angle -", "n", Color(0xFFD7CCC8), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("n") }
            }
            Spacer(Modifier.height(8.dp))

            Row(Modifier.fillMaxWidth(), Arrangement.SpaceEvenly) {
                MutedButton(
                    "Compass Mode", "C", Color(0xFFFFCCBC), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("C") }
                MutedButton(
                    "Search Mode", "A", Color(0xFFFFCCBC), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("A") }
            }
            Spacer(Modifier.height(8.dp))

            Row(Modifier.fillMaxWidth(), Arrangement.Center) {
                MutedButton(
                    "Set Heading", "h", Color(0xFFB3E5FC), textStyle, Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("h") }
            }
            Spacer(Modifier.height(8.dp))

            Row(Modifier.fillMaxWidth(), Arrangement.Center) {
                MutedButton(
                    "Stop All", "S", Color(0xFFB71C1C).copy(alpha = 0.8f),
                    textStyle.copy(color = Color.White), Modifier.weight(1f),
                    enabled = true
                ) { guardedSendCommand("S") }
            }
            Spacer(Modifier.height(16.dp))

            if (isConnected) {
                Text(
                    "Feedback:",
                    style = MaterialTheme.typography.bodyMedium.copy(
                        fontWeight = FontWeight.Bold,
                        fontSize = 18.sp
                    ),
                    modifier = Modifier.padding(top = 12.dp)
                )
                LazyColumn(
                    Modifier
                        .fillMaxWidth()
                        .weight(1f)
                        .background(Color.White)
                        .padding(4.dp)
                ) {
                    items(incomingMessages.size) { idx ->
                        Text(
                            incomingMessages[idx],
                            style = MaterialTheme.typography.bodySmall.copy(fontSize = 16.sp)
                        )
                    }
                }
            }
        }

        if (showNotConnectedPopup) {
            AlertDialog(
                onDismissRequest = { showNotConnectedPopup = false },
                title = { Text("Not connected", fontWeight = FontWeight.Bold, fontSize = 20.sp) },
                text = { Text("Connect to a Sonar Pole before using these functions.") },
                confirmButton = {
                    Button(onClick = { showNotConnectedPopup = false }) {
                        Text("OK")
                    }
                }
            )
        }

        if (showFeedbackDialog && manualCommandResult.isNotBlank()) {
            AlertDialog(
                onDismissRequest = { showFeedbackDialog = false },
                title = { Text("Command feedback") },
                text = { Text(manualCommandResult) },
                confirmButton = {
                    Button(onClick = { showFeedbackDialog = false }) { Text("OK") }
                }
            )
        }

        if (showManualCommandDialog) {
            ManualCommandDialog(
                onSend = { cmd ->
                    recentCommands = (listOf(cmd) + recentCommands).take(10)
                    lastManualCommand = cmd
                    manualCommandResult = "Waiting..."
                    sendCommand(cmd)
                },
                onDismiss = { showManualCommandDialog = false },
                recentCommands = recentCommands
            )
        }
    }
}

@Composable
fun MutedButton(
    label: String,
    command: String,
    color: Color,
    textStyle: TextStyle,
    modifier: Modifier = Modifier,
    enabled: Boolean = true,
    onClick: () -> Unit
) {
    Button(
        onClick = onClick,
        enabled = enabled,
        modifier = modifier
            .padding(4.dp)
            .height(48.dp),
        colors = ButtonDefaults.buttonColors(containerColor = color)
    ) {
        Text(label, style = textStyle, maxLines = 1)
    }
}

@Composable
fun SettingsDialog(
    onCommand: (String) -> Unit,
    onSendValueCommand: (String, String) -> Unit,
    onDismiss: () -> Unit,
    magnetometerEnabled: Boolean?,
    setMagnetometerEnabled: (Boolean) -> Unit,
    lastMessage: String,
    isConnected: Boolean,
    showManualCommandDialog: Boolean,
    onShowManualCommandDialog: (Boolean) -> Unit,
    recentCommands: List<String>,
    onAddManualCommand: (String) -> Unit,
    onClearCommands: () -> Unit,
    manualCommandResult: String
) {
    var showValueDialog by remember { mutableStateOf<String?>(null) }
    var showMagnetDialog by remember { mutableStateOf(false) }
    var showNotConnectedPopup by remember { mutableStateOf(false) }

    fun checkConnected(action: () -> Unit) {
        if (isConnected) action() else showNotConnectedPopup = true
    }

    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text(
                "Settings",
                fontSize = 26.sp,
                fontWeight = FontWeight.Bold,
                modifier = Modifier.padding(bottom = 8.dp)
            )
        },
        text = {
            Box(
                Modifier
                    .width(350.dp)
                    .heightIn(min = 520.dp, max = 700.dp)
            ) {
                val buttonColors = listOf(
                    Color(0xFFB0BEC5), // BlueGray
                    Color(0xFFC8E6C9), // Green
                    Color(0xFFD7CCC8), // BrownGray
                    Color(0xFFFFF9C4), // Yellow
                    Color(0xFFB3E5FC), // Cyan
                    Color(0xFFFFCCBC), // Orange
                    Color(0xFFF5F5F5), // Light Gray
                )
                val textStyle = MaterialTheme.typography.titleMedium.copy(
                    fontSize = 20.sp, fontWeight = FontWeight.Bold, color = Color(0xFF222222)
                )
                Column(
                    Modifier.fillMaxWidth(),
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Row(Modifier.fillMaxWidth(), Arrangement.SpaceEvenly) {
                        MutedButton(
                            "Kp", "P", buttonColors[3], textStyle, Modifier.weight(1f)
                        ) { checkConnected { showValueDialog = "Kp" } }
                        MutedButton(
                            "Ki", "I", buttonColors[3], textStyle, Modifier.weight(1f)
                        ) { checkConnected { showValueDialog = "Ki" } }
                        MutedButton(
                            "Kd", "D", buttonColors[3], textStyle, Modifier.weight(1f)
                        ) { checkConnected { showValueDialog = "Kd" } }
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(Modifier.fillMaxWidth(), Arrangement.SpaceEvenly) {
                        MutedButton(
                            "Set Threshold", "T", buttonColors[4], textStyle, Modifier.weight(1f)
                        ) { checkConnected { showValueDialog = "T" } }
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(Modifier.fillMaxWidth(), Arrangement.Center) {
                        MutedButton(
                            "Compass Calibration",
                            "Q", buttonColors[6], textStyle, Modifier.fillMaxWidth().height(48.dp)
                        ) { checkConnected { onCommand("Q") } }
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(Modifier.fillMaxWidth(), Arrangement.Center) {
                        MutedButton(
                            if (magnetometerEnabled == true) "Compass. ON" else "Compass. OFF",
                            "X",
                            if (magnetometerEnabled == true) Color(0xFFC8E6C9)
                            else Color(0xFFB71C1C).copy(alpha = 0.5f),
                            textStyle,
                            Modifier.fillMaxWidth().height(48.dp)
                        ) { checkConnected { showMagnetDialog = true } }
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(Modifier.fillMaxWidth(), Arrangement.Center) {
                        MutedButton(
                            "Read Settings", "O", buttonColors[0], textStyle, Modifier.weight(1f)
                        ) { checkConnected { onCommand("O") } }
                    }
                    Spacer(Modifier.height(8.dp))
                    Row(Modifier.fillMaxWidth(), Arrangement.Center) {
                        MutedButton(
                            "Manual Command", "", buttonColors[5], textStyle, Modifier.weight(1f)
                        ) { checkConnected { onShowManualCommandDialog(true) } }
                    }
                    if (recentCommands.isNotEmpty()) {
                        Column(
                            Modifier.padding(top = 8.dp),
                            horizontalAlignment = Alignment.Start
                        ) {
                            Text("Recent commands:", fontWeight = FontWeight.Bold, fontSize = 14.sp)
                            recentCommands.forEach {
                                Text(it, fontSize = 13.sp, color = Color.Gray)
                            }
                        }
                    }
                    if (manualCommandResult.isNotBlank() && manualCommandResult != "Waiting...") {
                        Text(
                            "Response: $manualCommandResult",
                            style = MaterialTheme.typography.bodyMedium.copy(fontWeight = FontWeight.Bold),
                            modifier = Modifier.padding(top = 4.dp)
                        )
                    }
                }
                when (showValueDialog) {
                    "Kp" -> ValueInputDialog(
                        title = "Enter Kp value",
                        onValueSend = { value ->
                            onSendValueCommand("P", value)
                            onCommand("O")
                        },
                        onDismiss = { showValueDialog = null }
                    )
                    "Ki" -> ValueInputDialog(
                        title = "Enter Ki value",
                        onValueSend = { value ->
                            onSendValueCommand("I", value)
                            onCommand("O")
                        },
                        onDismiss = { showValueDialog = null }
                    )
                    "Kd" -> ValueInputDialog(
                        title = "Enter Kd value",
                        onValueSend = { value ->
                            onSendValueCommand("D", value)
                            onCommand("O")
                        },
                        onDismiss = { showValueDialog = null }
                    )
                    "T" -> ValueInputDialog(
                        title = "Enter Threshold value",
                        onValueSend = { value ->
                            onSendValueCommand("T", value)
                            onCommand("O")
                        },
                        onDismiss = { showValueDialog = null }
                    )
                }
                if (showMagnetDialog) {
                    AlertDialog(
                        onDismissRequest = { showMagnetDialog = false },
                        title = { Text("Compass Function", fontWeight = FontWeight.Bold) },
                        text = { Text("Activate or deactivate the compass? This will remove all compass functions.") },
                        confirmButton = {
                            Button(
                                onClick = {
                                    onCommand("X")
                                    setMagnetometerEnabled(true)
                                    showMagnetDialog = false
                                },
                                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFC8E6C9))
                            ) { Text("Activate", fontWeight = FontWeight.Bold) }
                        },
                        dismissButton = {
                            Button(
                                onClick = {
                                    onCommand("X")
                                    setMagnetometerEnabled(false)
                                    showMagnetDialog = false
                                },
                                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFB71C1C).copy(alpha = 0.8f))
                            ) { Text("Deactivate", fontWeight = FontWeight.Bold, color = Color.White) }
                        }
                    )
                }
                if (showNotConnectedPopup) {
                    AlertDialog(
                        onDismissRequest = { showNotConnectedPopup = false },
                        title = { Text("Not connected", fontWeight = FontWeight.Bold, fontSize = 20.sp) },
                        text = { Text("Connect to a Sonar Pole before using these functions.") },
                        confirmButton = {
                            Button(onClick = { showNotConnectedPopup = false }) {
                                Text("OK")
                            }
                        }
                    )
                }
            }
        },
        confirmButton = {
            Row(
                Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Button(
                    onClick = onClearCommands,
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFEEEEEE))
                ) {
                    Text("Clear", fontWeight = FontWeight.Bold, color = Color(0xFF444444))
                }
                Button(
                    onClick = onDismiss,
                    colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFEEEEEE))
                ) {
                    Text(
                        "Close",
                        fontSize = 18.sp,
                        fontWeight = FontWeight.Bold,
                        color = Color(0xFF444444)
                    )
                }
            }
        }
    )
}

@Composable
fun ValueInputDialog(
    title: String,
    onValueSend: (String) -> Unit,
    onDismiss: () -> Unit
) {
    var inputValue by remember { mutableStateOf("") }
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text(title, fontWeight = FontWeight.Bold, fontSize = 22.sp) },
        text = {
            Column {
                OutlinedTextField(
                    value = inputValue,
                    onValueChange = { inputValue = it },
                    label = { Text("Enter value") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth()
                )
            }
        },
        confirmButton = {
            Button(
                onClick = {
                    onValueSend(inputValue)
                    onDismiss()
                },
                enabled = inputValue.isNotBlank()
            ) {
                Text("Send", fontWeight = FontWeight.Bold)
            }
        },
        dismissButton = {
            Button(onClick = onDismiss) {
                Text("Cancel")
            }
        }
    )
}

@Composable
fun ManualCommandDialog(
    onSend: (String) -> Unit,
    onDismiss: () -> Unit,
    recentCommands: List<String>
) {
    var cmd by remember { mutableStateOf("") }
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Send manual command", fontWeight = FontWeight.Bold, fontSize = 20.sp) },
        text = {
            Column {
                OutlinedTextField(
                    value = cmd,
                    onValueChange = { cmd = it },
                    label = { Text("Command") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth()
                )
                Spacer(Modifier.height(8.dp))
                Text("Last 10 commands:", fontWeight = FontWeight.Bold, fontSize = 15.sp)
                recentCommands.forEach {
                    Text(it, fontSize = 14.sp, color = Color.Gray)
                }
            }
        },
        confirmButton = {
            Button(
                onClick = {
                    if (cmd.isNotBlank()) onSend(cmd)
                    onDismiss()
                },
                enabled = cmd.isNotBlank()
            ) {
                Text("Send", fontWeight = FontWeight.Bold)
            }
        },
        dismissButton = {
            Button(onClick = onDismiss) { Text("Cancel") }
        }
    )
}

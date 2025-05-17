package com.example.whi_livepole // Byt till ditt paketnamn om det behövs!

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothSocket
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.core.app.ActivityCompat
import androidx.lifecycle.lifecycleScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import androidx.compose.ui.Alignment
import androidx.compose.ui.text.style.TextAlign
import java.util.*

class MainActivity : ComponentActivity() {
    private val bluetoothAdapter: BluetoothAdapter? by lazy { BluetoothAdapter.getDefaultAdapter() }
    private var bluetoothSocket: BluetoothSocket? = null
    private var outputStream: OutputStream? = null
    private var inputStream: InputStream? = null
    private var connectedDeviceName: String? = null

    private val hc05UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")
    private val incomingMessages = mutableStateListOf<String>()

    private fun hasBtPermission(): Boolean {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) == PackageManager.PERMISSION_GRANTED &&
                    ActivityCompat.checkSelfPermission(
                        this,
                        Manifest.permission.BLUETOOTH_SCAN
                    ) == PackageManager.PERMISSION_GRANTED
        } else {
            true
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val requiredPermissions = mutableListOf(
            Manifest.permission.BLUETOOTH,
            Manifest.permission.BLUETOOTH_ADMIN
        )
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            requiredPermissions.add(Manifest.permission.BLUETOOTH_CONNECT)
            requiredPermissions.add(Manifest.permission.BLUETOOTH_SCAN)
        }
        ActivityCompat.requestPermissions(this, requiredPermissions.toTypedArray(), 1)

        setContent {
            BluetoothScreen(
                scanDevices = { scanBluetoothDevices() },
                connectToDevice = { device -> connect(device) },
                sendCommand = { cmd -> sendBluetoothCommand(cmd) },
                getConnectedDevice = { connectedDeviceName },
                incomingMessages = incomingMessages,
            )
        }
    }

    private fun scanBluetoothDevices(): List<BluetoothDevice> {
        if (!hasBtPermission()) {
            incomingMessages.add("No Bluetooth access")
            return emptyList()
        }
        return try {
            bluetoothAdapter?.bondedDevices?.toList() ?: emptyList()
        } catch (e: SecurityException) {
            incomingMessages.add("Safety fault: ${e.message}")
            emptyList()
        }
    }

    @SuppressLint("MissingPermission")
    private fun connect(device: BluetoothDevice) {
        if (!hasBtPermission()) {
            incomingMessages.add("No Bluetooth Access")
            return
        }
        try {
            bluetoothSocket?.close()
        } catch (_: Exception) {}
        try {
            bluetoothSocket = device.createRfcommSocketToServiceRecord(hc05UUID)
            bluetoothSocket?.let {
                try {
                    it.connect()
                    outputStream = it.outputStream
                    inputStream = it.inputStream
                    connectedDeviceName = device.name
                    incomingMessages.clear()
                    incomingMessages.add("Connected to ${device.name}")
                    listenForIncomingMessages()
                } catch (e: SecurityException) {
                    connectedDeviceName = null
                    incomingMessages.add("Safety fault: ${e.message}")
                } catch (e: IOException) {
                    connectedDeviceName = null
                    incomingMessages.add("Connection Fault: ${e.message}")
                }
            }
        } catch (e: SecurityException) {
            connectedDeviceName = null
            incomingMessages.add("Safety fault: ${e.message}")
        }
    }

    private fun sendBluetoothCommand(cmd: String) {
        try {
            outputStream?.write(cmd.toByteArray())
            incomingMessages.add("BT Command: $cmd")
        } catch (e: Exception) {
            incomingMessages.add("Failed: ${e.message}")
        }
    }

    // Denna funktion hanterar permissions & SecurityException korrekt!
    private fun listenForIncomingMessages() {
        val input = inputStream ?: return
        val scope = lifecycleScope
        scope.launch(Dispatchers.IO) {
            val buffer = ByteArray(1024)
            while (true) {
                try {
                    val bytes = input.read(buffer)
                    if (bytes > 0) {
                        val msg = String(buffer, 0, bytes)
                        incomingMessages.add("Received: $msg")
                    }
                } catch (e: SecurityException) {
                    incomingMessages.add("Access Denied!")
                    break
                } catch (e: IOException) {
                    incomingMessages.add("Disconnected")
                    break
                }
            }
        }
    }
}

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

    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(top = 60.dp, start = 20.dp, end = 20.dp, bottom = 20.dp)
            .background(Color(0xFFF8F8F8)),
        horizontalAlignment = Alignment.CenterHorizontally  // <-- CENTRERAR!
    ) {
        Text(
            "WLI SONAR POLE",
            style = MaterialTheme.typography.headlineSmall,
            modifier = Modifier.fillMaxWidth(),
            textAlign = TextAlign.Center   // Du kan använda denna om du vill ha texten centrerad på raden
        )
        Spacer(Modifier.height(60.dp))

        if (connectedDevice == null) {
            Button(
                onClick = {
                    devices = scanDevices()
                    showDevices = true
                },
                modifier = Modifier.fillMaxWidth(0.7f) // Gör knappen lite bredare och centrerad
            ) {
                Text("Search Sonar Pole")
            }
            Spacer(Modifier.height(8.dp))
        }

        if (devices.isNotEmpty() && showDevices && connectedDevice == null) {
            Text("Connect:")
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
                        Text("${dev.name} (${dev.address})")
                    }
                }
            }
        }

        if (connectedDevice != null) {
            Text("Connected to: $connectedDevice", color = Color.Green)
            Spacer(Modifier.height(12.dp))

            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Button(onClick = { sendCommand("+") }) { Text("Manual +") }
                Button(onClick = { sendCommand("-") }) { Text("Manual -") }
                Button(onClick = { sendCommand("M") }) { Text("Manual") }
            }
            Spacer(Modifier.height(8.dp))
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Button(onClick = { sendCommand("U") }) { Text("Sweep angle +") }
                Button(onClick = { sendCommand("H") }) { Text("Sweep angle -") }
                Button(onClick = { sendCommand("A") }) { Text("Auto Search") }
            }
            Spacer(Modifier.height(8.dp))
            Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                Button(onClick = { sendCommand("N") }) { Text("Sweep angle +") }
                Button(onClick = { sendCommand("n") }) { Text("Sweep angle -") }
                Button(
                    onClick = { sendCommand("S") },
                    colors = ButtonDefaults.buttonColors(containerColor = Color.Red)
                ) { Text("Stop All", color = Color.White) }
            }
            Spacer(Modifier.height(12.dp))
            Text("Feedback:", style = MaterialTheme.typography.bodyMedium)
            LazyColumn(
                Modifier
                    .fillMaxWidth()
                    .weight(1f)
                    .background(Color.White)
                    .padding(4.dp)
            ) {
                items(incomingMessages.size) { idx ->
                    Text(incomingMessages[idx], style = MaterialTheme.typography.bodySmall)
                }
            }
        }
    }
}

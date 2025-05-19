package com.example.whi_livepole

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
import androidx.compose.material3.*
import androidx.core.app.ActivityCompat
import androidx.lifecycle.lifecycleScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.*
import androidx.compose.runtime.mutableStateListOf

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
        if (outputStream == null) {
            incomingMessages.add("No device connected!")
            return
        }
        try {
            outputStream?.write(cmd.toByteArray())
            incomingMessages.add("BT Command: $cmd")
        } catch (e: Exception) {
            incomingMessages.add("Failed: ${e.message}")
        }
    }


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

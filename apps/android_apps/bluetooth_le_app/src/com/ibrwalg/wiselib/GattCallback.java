package com.ibralg.wiselib;

import android.bluetooth.*;
import android.util.Log;

class GattCallback extends BluetoothGattCallback {

	// Callback triggered as a result of a remote characteristic notification.
	public void onCharacteristicChanged(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic) {
		Log.d("WiselibDebug", "onCharChnaged: " + characteristic.toString());
	}

	// Callback reporting the result of a characteristic read operation.
	public void onCharacteristicRead(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
		Log.d("WiselibDebug", "onCharRead: " + characteristic.toString());
	}

	// Callback indicating the result of a characteristic write operation.
	public void onCharacteristicWrite(BluetoothGatt gatt, BluetoothGattCharacteristic characteristic, int status) {
		Log.d("WiselibDebug", "onCharWrite: " + characteristic.toString());
	}

	// Callback indicating when GATT client has connected/disconnected to/from a remote GATT server.
	public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
		if(newState == BluetoothProfile.STATE_CONNECTED) {
			Log.d("WiselibDebug", "Connected: " + gatt.getDevice().getName() + " @ " + gatt.getDevice().getAddress() );
		} else if(newState == BluetoothProfile.STATE_DISCONNECTED) {
			Log.d("WiselibDebug", "Disconnected: " + gatt.getDevice().getName() + " @ " + gatt.getDevice().getAddress() );
		}

	}

	// Callback reporting the result of a descriptor read operation.
	public void onDescriptorRead(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
		Log.d("WiselibDebug", "onDescripRead " + descriptor.toString() );
	}

	// Callback indicating the result of a descriptor write operation.
	public void onDescriptorWrite(BluetoothGatt gatt, BluetoothGattDescriptor descriptor, int status) {
		Log.d("WiselibDebug", "onDescripWrite " + descriptor.toString() );
	}

	// Callback reporting the RSSI for a remote device connection.
	public void onReadRemoteRssi(BluetoothGatt gatt, int rssi, int status) {
		;
	}

	// Callback invoked when a reliable write transaction has been completed.
	public void onReliableWriteCompleted(BluetoothGatt gatt, int status) {
		;
	}

	// Callback invoked when the list of remote services, characteristics and descriptors for the remote device have been updated, ie new services have been discovered.
	public void onServicesDiscovered(BluetoothGatt gatt, int status) {
		;
	}

}

package com.ibralg.wiselib;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;
import android.content.pm.PackageManager;
import android.content.Context;
import android.content.Intent;
import android.bluetooth.*;

public class WiselibActivity extends Activity implements BluetoothAdapter.LeScanCallback {

   private static final long RESTART_SCAN_MILLIS = 1100;

	public native int exampleapp(WiselibActivity wiselibActivity);
	public native void onBleDataReceive(long macAddr, byte[] data, int rssi);

	private android.net.wifi.WifiManager.MulticastLock lock;
	private android.os.Handler handler = new android.os.Handler();

	private BluetoothAdapter bluetoothAdapter;
   private Runnable bluetoothRestarter;
   private boolean isBluetoothLeEnabled = false;

	/**
	 * Called when the activity is created
    * or restarted and shall restore itself
    */
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);

		final BluetoothAdapter.LeScanCallback cb = this;
      bluetoothRestarter = new Runnable() {
			@Override
			public void run() {
				bluetoothAdapter.stopLeScan(cb);
				bluetoothAdapter.startLeScan(cb);
				handler.postDelayed(bluetoothRestarter, RESTART_SCAN_MILLIS);
			}
		};

		handler.postDelayed(new Runnable() {
			public void run() {
				setUp();
			}
		}, 100);
	}

	/**
	 * Called by onCreate() after 100ms
	 */
	private void setUp() {
		android.net.wifi.WifiManager wifi = (android.net.wifi.WifiManager) getSystemService(android.content.Context.WIFI_SERVICE);
		lock = wifi.createMulticastLock("wiseliblock");
		lock.setReferenceCounted(true);
		lock.acquire();
		exampleapp(this);
	}

	/**
	 * Called by Android when activiy is closed (e.g. when the user
	 * presses the back key)
	 */
	@Override
	protected void onStop() {
		super.onStop();
		lock.release();
		disableBluetoothLe();
	}


/*****************  Interface to native code for Bluetooth Low Energy Radio ***********************/

	/**
	 * Enables BLE and starts scanning
	 * @return true, if BLE is available and enabled; false otherwise
	 */
	public boolean enableBluetoothLe() {
      if (isBluetoothLeEnabled) return true;

		// check if BLE is present
		if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
			Toast.makeText(this, "Bluetooth Low Energy not supported", Toast.LENGTH_SHORT).show();
			return false;
		}

		// check if BLE is enabled
		final BluetoothManager bluetoothManager = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
		bluetoothAdapter = bluetoothManager.getAdapter();
		if (bluetoothAdapter == null || !bluetoothAdapter.isEnabled()) {
			Toast.makeText(this, "Please enable Bluetooth and restart the app", Toast.LENGTH_LONG).show();
			return false;
		}

		// scan for BLE devices and restart scan after a specific timeout
		bluetoothAdapter.startLeScan(this);
      handler.postDelayed(bluetoothRestarter, RESTART_SCAN_MILLIS);

		// code will continue in onLeScan()
		Log.i("WiselibDebug", "Scan started");
      isBluetoothLeEnabled = true;
		return true;
	}

	/**
	 * Stops BLE scanning
	 * @return true if done successfully
	 */
	public boolean disableBluetoothLe() {
		if(!isBluetoothLeEnabled) return true;

		if(bluetoothAdapter != null) {
         handler.removeCallbacks(bluetoothRestarter);
			bluetoothAdapter.stopLeScan(this);
			Log.i("WiselibDebug", "Scan stopped");
			isBluetoothLeEnabled = false;
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Format:
	 * -  9 Bytes fixed header: 02 01 1a 1a ff 4c 00 02 15
	 * - 16 Bytes UUID of beacon
	 * -  2 Bytes major ID
	 * -  2 Bytes minor ID
	 * -  1 Byte: 2s complemnt of calibrated Tx power
	 * = 30 Bytes total
	 */
	private void printBeaconInfo(BluetoothDevice device, byte[] scanRecord, int rssi) {
		Log.d("WiselibDebug", "Found: " + device.getName() + " @ " + device.getAddress() );
		Log.d("WiselibDebug", String.format("Mac=%X", macAddrFromString(device.getAddress())) );

		if(scanRecord.length < 30) return;
		
		// printt header
		String header = " * Header = ";
		for(int i=0; i<9; i++) {
			int unsignedVal;
			if(scanRecord[i] < 0)	unsignedVal = 256 + scanRecord[i];  // remove sign
			else			unsignedVal = scanRecord[i];
			header += "." + Integer.toString( unsignedVal, 16);
		}
		Log.d("WiselibDebug", header);
		
		// print UUID
		String uuid = " * UUID = ";
		for(int i=9; i<25; i++) {
			int unsignedVal;
			if(scanRecord[i] < 0)	unsignedVal = 256 + scanRecord[i];  // remove sign
			else			unsignedVal = scanRecord[i];
			uuid += "." + Integer.toString( unsignedVal, 16);
		}
		Log.d("WiselibDebug", uuid);

		// print major and minor number
		int major = scanRecord[25] << 8 | scanRecord[26];
		int minor = scanRecord[27] << 8 | scanRecord[28];
		Log.d("WiselibDebug", " * Major: " + major + ", Minor: " + minor);

		// print tx power
		int txPower = scanRecord[29];
		if (txPower < 0) txPower = scanRecord[29] + 256;
		Log.d("WiselibDebug", " * TxPower: " + txPower);

		// print length
		Log.d("WiselibDebug", " * Total Bytes: " + scanRecord.length);
		Log.d("WiselibDebug", "Rssi: " + rssi);
	}

	/**
	 * Converts a mac address like "00:11:22:33:44:55" to a 64 bit value like 0x001122334455
    */
	public long macAddrFromString(String mac) {
		long ret = 0x00L;
		for(int i=0; i<6; i++) {
			long val = (Character.digit(mac.charAt(i*3), 16) << 4 | Character.digit(mac.charAt(i*3+1), 16) );
			ret = (ret << 8) | val;
		}
		return ret;
	}

   //The RSSI value for the remote device as reported by the Bluetooth hardware. 0 if no RSSI value is available.
	@Override
	public void onLeScan(BluetoothDevice device, int rssi, byte[] scanRecord) {
		//printBeaconInfo(device, scanRecord, rssi);
		onBleDataReceive(macAddrFromString(device.getAddress()), scanRecord, rssi);
	}

/********************************************************************************************************/

	static {
		System.loadLibrary("example_app");
	}
}

package ro.fsck.wiselib;

import android.app.Activity;
import android.os.Bundle;

public class TestWiselibActivity extends Activity {

	public native int exampleapp();

	android.net.wifi.WifiManager.MulticastLock lock;
	android.os.Handler handler = new android.os.Handler();

	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		


		handler.postDelayed(new Runnable() {
			public void run() {
				setUp();
			}
		}, 1000);


	}

	/** Called when the activity is first created. */

	private void setUp() {
		android.net.wifi.WifiManager wifi = (android.net.wifi.WifiManager) getSystemService(android.content.Context.WIFI_SERVICE);
		lock = wifi.createMulticastLock("wiseliblock");
		lock.setReferenceCounted(true);
		lock.acquire();
		exampleapp();


	}

	@Override
	protected void onStart() {
		super.onStart();
	}

	@Override
	protected void onStop() {
		lock.release();
		super.onStop();
	}

	static {
		System.loadLibrary("example_app");
	}
}

package org.wiselib.zeroconf;

import java.io.IOException;
import java.util.HashMap;

import javax.jmdns.JmDNS;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;

import android.util.Log;

public class Zeroconf {
	private JmDNS jmdns = null;
	private ServiceInfo serviceInfo;
	String nodes = "";

	public Zeroconf(String service, int port) {

		try {
			jmdns = JmDNS.create();

		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		String service_key = "description";
		String service_text = "Wiselib service";
		HashMap<String, byte[]> properties = new HashMap<String, byte[]>();
		properties.put(service_key, service_text.getBytes());

		jmdns.addServiceListener(service, new ZeroconfServiceListener());

		try {
			serviceInfo = ServiceInfo
					.create(service, java.net.InetAddress.getLocalHost()
							.getHostName(), port, 0, 0, true, properties);
			jmdns.registerService(serviceInfo);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public void stopZeroconf() {
		if (jmdns != null) {
			jmdns.unregisterAllServices();
			try {
				jmdns.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
			jmdns = null;
		}
	}

	public synchronized String getNodes() {
		return this.nodes;
	}

	public synchronized void addNode(String node) {
		this.nodes += node + ";";
	}

	public synchronized void clearNodes() {
		this.nodes = "";
	}

	public class ZeroconfServiceListener implements ServiceListener {

		public void serviceAdded(ServiceEvent arg0) {
			Log.d("added", arg0.getName() + " addr: "
					+ arg0.getInfo().getHostAddresses()[0].toString());
			jmdns.requestServiceInfo(arg0.getType(), arg0.getName(), true, 1);

		}

		public void serviceRemoved(ServiceEvent arg0) {
			Log.d("removed", "Service removed: " + arg0.getName() + " addr: "
					+ arg0.getInfo().getHostAddresses()[0].toString());
			addNode("D" + arg0.getInfo().getHostAddresses()[0].toString());

		}

		public void serviceResolved(ServiceEvent arg0) {
			Log.d("resolved", "Service resolved: "
					+ arg0.getInfo().getQualifiedName() + " port:"
					+ arg0.getInfo().getPort() + " addr: "
					+ arg0.getInfo().getHostAddresses()[0].toString());
			addNode("A" + arg0.getInfo().getHostAddresses()[0].toString());

		}

	}
}

package com.lxyppc;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import com.lxyppc.SVDDevice.Group;
import com.lxyppc.SVDDevice.Interrupt;
import com.lxyppc.SVDDevice.Peripheral;

public class SVDMerge {
	private Map<String, List<SVDDevice> > mAllGroup = new HashMap<String, List<SVDDevice> >();
	private List<String> devices = new ArrayList<String>();
	
	public SVDMerge(){	
	}
	
	public void addDevice(SVDDevice dev){
		Iterator<SVDDevice.Group> it = dev.mGroups.iterator();
		while(it.hasNext()){
			SVDDevice.Group gp = it.next();
			if(mAllGroup.containsKey(gp.mName)){
				mAllGroup.get(gp.mName).add(dev);
			}else{
				ArrayList<SVDDevice> l = new ArrayList<SVDDevice>();
				l.add(dev);
				mAllGroup.put(gp.mName, l);
			}
		}
		devices.add(dev.name);
	}
	
	String genOutputFolder(){
		String r = "";
		int pos = 0;
		char c = '\0';
		boolean con = true;
		do{
			Iterator<String> it = devices.iterator();
			while(it.hasNext()){
				String t = it.next();
				if(pos<t.length()){
					if(c == '\0'){
						c = t.toLowerCase().charAt(pos);
					}else{
						if(c != t.toLowerCase().charAt(pos)){
							con = false;
							break;
						}
					}
				}else{
					con = false;
					break;
				}
			}
			pos++;
			if(con) r+=c;
			c = '\0';
		}while(con);
		r += "x";
		if(r.length() == 0){
			r = "output";
		}
		
		String s = "Create the SFR for ";
		Iterator<String> it = devices.iterator();
		while(it.hasNext()){
			s += it.next();
			if(it.hasNext()) s+=",";
			s+=" ";
		}
		s += "in folder ";
		SVDDevice.log(s + r);
		File f = new File(r);
		f.mkdir();
		r += "/";
		return r;
	}
	
	public void ouput(){
		Iterator<String> it = mAllGroup.keySet().iterator();
		String prefix = genOutputFolder();
		while(it.hasNext()){
			String gpName = it.next();
			List<SVDDevice> devs = mAllGroup.get(gpName);
			List< Peripheral> ps = new ArrayList<SVDDevice.Peripheral>();
			Iterator<SVDDevice> devIt = devs.iterator();
			while(devIt.hasNext()){
				SVDDevice dev = devIt.next();
				dev.filePrefix = prefix;
				Group gp = dev.findGroup(gpName);
				if(gp != null){
					ps.add(gp.getMaxPeripheral());
				}
			}
			SVDDevice.setCompMethod(Peripheral.REGS);
			Collections.sort(ps, Collections.reverseOrder());
			int seq = 0;
			int i;
			Peripheral p = null;
			for(i = 0;i<ps.size();i++){
				Peripheral t = ps.get(i);
				if(p == null){
					p = t;
				}else{
					if(p.compareRegs(t) > 1){
						p.parent().parent().output(p.groupName , (seq>0?""+seq:""), deviceInfo(p.parent().parent().name));
						seq++;;
					}else{
						break;
					}
					p = t;
				}
			}
			if(p!=null){
				p.parent().parent().setIRQs(compositeIRQ(ps.indexOf(p),ps));
				p.parent().parent().setPeripherals(compositePer(ps.indexOf(p),ps));
				p.parent().parent().output(p.groupName , (seq>0?""+seq:""),deviceInfo(ps.indexOf(p),ps));
				p.parent().parent().setPeripherals(null);
				p.parent().parent().setIRQs(null);
			}
		}
	}
	
	List<Peripheral> compositePer(int start, List< Peripheral> ps){
		List<Peripheral> r = new ArrayList<Peripheral>();
		List<Peripheral> rx = new ArrayList<Peripheral>();
		Iterator<Peripheral> it = ps.iterator();
		while(it.hasNext()){
			r.addAll(it.next().parent().getPeripherals());
		}
		SVDDevice.setCompMethod(Peripheral.BASE_ADDR);
		Collections.sort(r);
		it = r.iterator();
		Peripheral lastPer = null;
		while(it.hasNext()){
			Peripheral per = it.next();
			if(lastPer == null){
				rx.add(per);
			}else{
				if(lastPer.name.compareTo(per.name) != 0){
					rx.add(per);
				}
			}
			lastPer = per;
		}
		return rx;
	}
	
	List<Interrupt> compositeIRQ(int start, List< Peripheral> ps){
		List<Interrupt> r = new ArrayList<Interrupt>();
		Iterator<Peripheral> it = ps.iterator();
		while(it.hasNext()){
			r.addAll(it.next().parent().getIRQs());
		}
		Iterator<Interrupt> itIRQ = r.iterator();
		Collections.sort(r);
		List<Interrupt> rx = new ArrayList<Interrupt>();
		itIRQ = r.iterator();
		Interrupt lastIRQ = null;
		while(itIRQ.hasNext()){
			Interrupt irq = itIRQ.next();
			if(lastIRQ == null){
				rx.add(irq);
			}else{
				if(lastIRQ.name.compareTo(irq.name) != 0){
					rx.add(irq);
				}
			}
			lastIRQ = irq;
		}
		return rx;
	}
	
	
	public String deviceInfo(String dev){
		return "/*  supported devices: " + dev + "  */";
	}
	public String deviceInfo(int start, List< Peripheral> ps){
		String r = "/*  supported devices: ";
		while(start<ps.size()){
			r += ps.get(start).parent().parent().name + ", ";
			start++;
		}
		r+= "  */";
		return r;
	}
}

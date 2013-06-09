package com.lxyppc;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;

public class SVDDevice {
	public String name = "name";
	public String description = "description";
	public List<Group> mGroups = new ArrayList<Group>();
	
	private String mDefSize = "0x20";
	private String mDefResetValue = "0x0";
	private String mDefResetMask = "0xFFFFFFFF";
	private String mFilename = null;
	public static final String NEWLINE = "\r\n";
	
	public SVDDevice(){
	}
	
	public SVDDevice(String filename){
		mFilename = filename;
	}
	
	boolean load(String filename){
		mFilename = filename;
		return load();
	}
	
	boolean load(){
		if(mFilename == null) {
			log("File name not set");
			return false;
		}
		try {
			
			DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
			DocumentBuilder builder = factory.newDocumentBuilder();   
            Document doc = builder.parse(mFilename);   
            doc.normalize(); 
            
            NodeList links = doc.getElementsByTagName("device");
            if(links.getLength()>0){
            	Element ele = (Element) links.item(0);
            	PorcessDevice(ele);
            }
		} catch (Exception e) {
			log(e.toString());
			return false;
		}
		
		return true;
	}
	
	boolean PorcessDevice(Element element){
		name = get_tag(element, "name", "def_name");
		description = get_tag(element, "description", "def_description");
		mDefSize = get_tag(element, "size", mDefSize);
		mDefResetValue = get_tag(element, "resetValue", mDefResetValue);
		mDefResetMask = get_tag(element, "resetMask", mDefResetMask);
		
		NodeList nodes = get_tag(element, "peripheral");
		for(int i=0; i< nodes.getLength(); i++){
			Element ele =  (Element)nodes.item(i);
			PorcessPeripheral(ele);
		}
		return true;
	}
	
	boolean PorcessPeripheral(Element element){
		Peripheral per = new Peripheral();
		per.parse(element);
		return true;
	}
	
	public Peripheral getPeripheral(String name){
        Iterator<Group> it = mGroups.iterator();
        for (; it.hasNext();) {
        	Peripheral r = it.next().getPeripheral(name);
        	if(r != null) return r;
        }
		return null;
	}
	
	public Group findGroup(String name){
		Iterator<Group> it = mGroups.iterator();
		while(it.hasNext()){
			Group gp = it.next();
			if(gp.mName.compareTo(name) == 0){
				return gp;
			}
		}
		return null;
	}
	
	public Group getGroup(String name){
		Group gp = findGroup(name);
		if(gp != null) return gp;
		mGroups.add(new Group(name,this));
		return getGroup(name);
	}
	
	public class Group{
		public String mName;
		public String mPostfix = "";
		public Group(String name, SVDDevice parent){
			this.mName = name;
			mParent = parent;
		}
		public String gpName(){
			return mName + mPostfix;
		}
		private List<Peripheral> mPeripherals = new ArrayList<Peripheral>();
		private SVDDevice mParent;
		public SVDDevice parent(){
			return mParent;
		}
		
		public void addPeripheral(Peripheral p){
			mPeripherals.add(p);
		}
		
		public Peripheral getPeripheral(String name){
			Iterator<Peripheral> it = mPeripherals.iterator();
			while(it.hasNext()){
				Peripheral p = it.next();
				if(p.name.compareTo(name)==0){
					return p;
				}
			}
			return null;
		}
		
		public List<Peripheral> getPeripherals(){
			return mPeripherals;
		}
		
		public Peripheral getMaxPeripheral(){
			List<Peripheral> list = getPeripherals();
			setCompMethod(Peripheral.REGS);
			Collections.sort(list, Collections.reverseOrder());
			return list.get(0);
		}
		
		public boolean checkMaxPeripheral(){
			Peripheral p = getMaxPeripheral();
			Iterator<Peripheral> it = mPeripherals.iterator();
			boolean res = true;
			while(it.hasNext()){
				Peripheral t = it.next();
				if(p.compareRegs(t) > 1){
					//log(p.name + " can not represent " + t.name);
					res = false;
				}
			}
			return res;
		}
		
		public List<Peripheral> getPeripheralSortByBase(){
			List<Peripheral> list = getPeripherals();
			setCompMethod(Peripheral.BASE_ADDR);
			Collections.sort(list);
			return list;
		}
		
		public List<Interrupt> getIRQs(){
			List<Peripheral> list = getPeripherals();
			Iterator<Peripheral> it = list.iterator();
			List<Interrupt> r = new ArrayList<Interrupt>();
			while(it.hasNext()){
				r.addAll(it.next().interrupts);
			}
			Collections.sort(r);
			return r;
		}
	}
	
	private List<Interrupt> mExtIRQs = null;
	public void setIRQs(List<Interrupt> irqs){
		mExtIRQs = irqs;
	}
	
	private List<Peripheral> mExtPers = null;
	public void setPeripherals(List<Peripheral> pers){
		mExtPers = pers;
	}
	
	public List<Interrupt> getIRQs(){
		Iterator<Group> it = mGroups.iterator();
		List<Interrupt> r = new ArrayList<Interrupt>();
		while(it.hasNext()){
			r.addAll(it.next().getIRQs());
		}
		Collections.sort(r);
		return r;
	}
	
	static private int comp_method = Peripheral.BASE_ADDR;
	static public void setCompMethod(int method){
		comp_method = method;
	}
	public class Peripheral implements Comparable<Peripheral> {
		private Group mParent;
		public static final int REGS = 0;
		public static final int BASE_ADDR = 1;
		public static final int IRQ_NUM = 2;
		public String name;
		public String description;
		public String baseAddress;
		public String groupName = "def_group";
		public List<Register> registers = new ArrayList<Register>();
		public List<Interrupt> interrupts = new ArrayList<Interrupt>();
		public Peripheral(){}

		String derivedFrom = "";
		public Group parent() {
			return mParent;
		}
		
		public void parse(Element element){
			String derived = get_attr(element, "derivedFrom");
			if(derived.length() > 0){
				Peripheral other = getPeripheral(derived);
				if(other != null){
					mParent = other.mParent;
					description = other.description;
					baseAddress = other.baseAddress;
					derivedFrom = other.name;
					groupName = other.groupName;
					registers = new ArrayList<Register>(other.registers);
				}
			}
			String v = get_tag(element, "name", "");
			if(v.length()>0) name = v;
			v = get_tag(element, "description", "");
			if(v.length()>0) description = v;
			v = get_tag(element, "baseAddress", "");
			if(v.length()>0) baseAddress = v;
			v = get_tag(element, "groupName", "");
			if(v.length()>0) groupName = v;
			Group gp = getGroup(groupName);
			mParent = gp;
			gp.addPeripheral(this);
			
			NodeList nodes = get_tag(element,"register");
			if(nodes.getLength()>0){
				registers.clear();
			}
			for(int i=0;i<nodes.getLength();i++){
				Element ele = (Element)nodes.item(i);
				Register reg = new Register();
				reg.parse(ele);
				registers.add(reg);
			}
			Collections.sort(registers);
			
			if(has_tag(element,"interrupt")){
				nodes = get_tag(element,"interrupt");
				for(int i=0;i<nodes.getLength();i++){
					Element ele = (Element)nodes.item(i);
					Interrupt interrupt = new Interrupt();
					interrupt.parse(ele);
					interrupts.add(interrupt);
				}
			}
			Collections.sort(interrupts);
		}
		
		public String typeName(){
			if(derivedFrom.length() > 0){
				return derivedFrom;
			}
			return name;
		}
		
		public int compareBaseAddress(Peripheral other){
			long v1 = 0;
			long v2 = 0;
			v1 = parseLong(baseAddress, 16);
			v2 = parseLong(other.baseAddress, 16);
			if(v1 > v2){
				return 1;
			}else if(v1<v2){
				return -1;
			}
			return 0;
		}
		
		public int compareIRQ(Peripheral other){
			long v1 = 0;
			long v2 = 0;
			
			v1 = interrupts.size()>0 ? parseLong(interrupts.get(0).value):0xfffff;
			v1 = other.interrupts.size()>0 ? parseLong(other.interrupts.get(0).value):0xfffff;
			if(v1 > v2){
				return 1;
			}else if(v1<v2){
				return -1;
			}
			return 0;
		}
		
		
		// 0  - equal
		// -1 - has less regs than other
		// 1  - has more regs than other
		// 2 - has different regs with other
		public int compareRegs(Peripheral other){ return compareRegs(other, false);}
		public int compareRegs(Peripheral other, boolean showDetail){
			Map<String, Object> l1 = new HashMap<String, Object>();
			Map<String, Object> r1 = new HashMap<String, Object>();
			Map<String, Object> r2 = new HashMap<String, Object>();
			Iterator<Register> it = registers.iterator();
			
			Map<Register, Object> x = new java.util.TreeMap<Register, Object>();
			x.put(new Register(), 1);
			while(it.hasNext()){
				String s = it.next().name;
				l1.put(s, s);
				r1.put(s, s);
			}
			
			it = other.registers.iterator();
			while(it.hasNext()){
				String s = it.next().name;
				r2.put(s, s);
				r1.remove(s);
			}
			
			Iterator<String> it2 = l1.keySet().iterator();
			while(it2.hasNext()){
				String s;
				s = it2.next();
				r2.remove(s);
			}
			
			if(r1.isEmpty() && r2.isEmpty()){
				return 0;
			}
			if(r1.isEmpty()){
				return -1;
			}
			if(r2.isEmpty()){
				return 1;
			}
			if(showDetail){
				String t = "";
				log("r1 remain");
				t = "";
				it2 = r1.keySet().iterator();
				while(it2.hasNext()){
					t += it2.next() + ",";
				}
				log(t);
				log("r2 remain");
				t = "";
				it2 = r2.keySet().iterator();
				while(it2.hasNext()){
					t += it2.next() + ",";
				}
				log(t);
			}
			return 2;
		}

		@Override
		public int compareTo(Peripheral o) {
			switch(comp_method){
			case REGS:
				return compareRegs(o);
			case IRQ_NUM:
				return compareIRQ(o);
			case BASE_ADDR:
			default:
				return compareBaseAddress(o);
			}
		}
	}
	
	public class Interrupt implements Comparable<Interrupt>{
		public Interrupt(){
			valid = false;
		}
		public Interrupt(Interrupt other){
			valid = other.valid;
			name = other.name;
			value = other.value;
			description = other.description;
		}
		public String name;
		public String value;
		public String description;
		public boolean valid;
		
		public void parse(Element element){
			valid = true;
			name = get_tag(element, "name", "def_name");
			value = get_tag(element, "value", "def_value");
			description = get_tag(element, "description", "def_description");
		}
		@Override
		public int compareTo(Interrupt arg0) {
			long v1 = valid ? parseLong(value) : 0xffff;
			long v2 = arg0.valid ? parseLong(arg0.value) : 0xffff;
			if(v1<v2)return -1;
			if(v1>v2)return 1;
			return 0;
		}
	}
	
	public class Register implements Comparable<Register>{
		public Register(){}
		public List<Field> fields = new ArrayList<Field>();
		public String name;
		public String displayName;
		public String description;
		public String addressOffset;
		public String size;
		public String access;
		public String resetValue;
		
		public void parse(Element element){
			name = get_tag(element,"name", "def_reg_name");
			displayName = get_tag(element,"displayName", "def_reg_display_name");
			description = get_tag(element,"description", "def_reg_desc");
			addressOffset = get_tag(element,"addressOffset", "0");
			size = get_tag(element,"size", mDefSize);
			access = get_tag(element,"access", "def_reg_access");
			resetValue = get_tag(element,"resetValue", mDefResetValue);
			NodeList fds = get_tag(element, "field");
			if(fds.getLength()>0){
				fields.clear();
			}
			for(int i=0;i<fds.getLength();i++){
				Field f = new Field();
				f.parse((Element)fds.item(i));
				fields.add(f);
			}
			Collections.sort(fields);
		}

		@Override
		public int compareTo(Register o) {
			long v1 = parseLong(addressOffset);
			long v2 = parseLong(o.addressOffset);
			return compare(v1,v2);
		}
	}
	
	public class Field implements Comparable<Field> {
		public String name;
		public String description;
		public String bitOffset;
		public String bitWidth;
		public Field(){}
		public void parse(Element element){
			name = get_tag(element,"name", "def_field_name");
			description = get_tag(element,"description", "def_field_desc");
			bitOffset = get_tag(element,"bitOffset", "0");
			bitWidth = get_tag(element,"bitWidth", "1");
		}
		@Override
		public int compareTo(Field arg0) {
			return compare(parseLong(bitOffset) , parseLong(arg0.bitOffset));
		}
	}
	
	public static String get_tag(Element element, String tag, String defaultValue){
		NodeList nodes = element.getElementsByTagName(tag);
		if(nodes.getLength() > 0){
			NodeList ns = nodes.item(0).getChildNodes();
			if(ns.getLength() > 0){
				defaultValue = ns.item(0).getNodeValue();
			}
		}
		return defaultValue;
	}
	
	public static String get_attr(Element element, String tag){
		return element.getAttribute(tag);
	}
	
	public static boolean has_tag(Element element, String tag){
		NodeList nodes = element.getElementsByTagName(tag);
		if(nodes.getLength() > 0){
			return true;
		}
		return false;
	}
	
	public static NodeList get_tag(Element element, String tag){
		return element.getElementsByTagName(tag);
	}
	
	public static long parseLong(String num){
		return parseLong(num,10);
	}
	
	public static int compare(long x, long y){
		if(x<y)return -1;
		if(x>y)return 1;
		return 0;
	}
	
	public static long parseLong(String num, int radix){
		if(num == null) return 0;
		if(num.startsWith("0x")){
			radix = 16;
			num = num.substring(2);
		}
		
		long r = 0;
		try{
			r =  Long.parseLong(num, radix);
		}catch(NumberFormatException e){
			//e.printStackTrace();
			log(num+"  " + radix);
		}
		//log(num + String.format("   %08x  ", r));
		return r;
	}
	
	public static String TAB = "    ";
	public static int COMMENT_INDEX = 50;
	public static int COMMENT_END = 128;
	
	void output_head(Group gp, boolean groupPerFile){
		if(!groupPerFile){
			writefile("");
			writefile("/* ================================================================================ */");
			writefile(moveTo("/* =====================       "+ gp.gpName(), 54) + "  =========================== */");
			writefile("/* ================================================================================ */");
			writefile("");
		}
		writefile(comment(TAB + "namespace " +  gp.gpName().toLowerCase() + "{", gp.getMaxPeripheral().description));
	}
	void output_tail(Group gp, boolean groupPerFile){
		writefile(TAB + "}//namespace " +  gp.gpName().toLowerCase() + "");
	}
	
	String moveTo(String str, int col){
		col = col - str.length();
		while(col>0){
			str += " ";
			col--;
		}
		return str;
	}
	
	String comment(String str, String comment){
		int col = COMMENT_INDEX - str.length();
		while(col>0){
			str += " ";
			col--;
		}
		comment = comment.replace('\n', ' ');
		while(comment.indexOf("  ") > 0)
		comment = comment.replace("  ", " ");
		str = str + "/*!<" + comment;
		col = COMMENT_END - str.length();
		while(col>0){
			str += " ";
			col--;
		}
		return str + "*/";
		
	}
	
	static String sizeType(String size){
		int s = (int)parseLong(size);
		if( s == 32){
			return "uint32_t";
		}else if(s == 16){
			return "uint16_t";
		}else if(s == 8){
			return "uint8_t";
		}  
		log("Unknown size, default to uint32_t");
		return "uint32_t";
	}
	
	static String qualifier(Register reg){
		boolean write = reg.access.contains("write");
		//boolean read = reg.access.contains("read");
		if(write){
			return "volatile";
		}
		return "const volatile";
	}
	
	void output(String reg_t, Field fd){
		String ident = TAB+TAB+TAB;
		writefile(comment(ident + "sfb_t<" + reg_t + ", "+fd.bitOffset + ", " + fd.bitWidth + "> " + fd.name + ";", fd.description));
	}
	
	void output(Register reg, boolean withField){
		String ident = TAB+TAB;
		String reg_t = reg.name.toLowerCase() + "_t";
		if(withField){
			writefile(comment(ident + "union " + reg_t + " {",reg.description));
			writefile(comment(ident+TAB + "__SFR("+reg_t+ ", "+sizeType(reg.size)+", 0)", "Address offset: " + reg.addressOffset + " Reset value: " + reg.resetValue));
			Iterator<Field> it = reg.fields.iterator();
			while(it.hasNext()){
				output(reg_t, it.next());
			}
					
			writefile(ident + "};");
			writefile("");
		}else{
			ident += TAB;
			writefile(comment(ident + qualifier(reg) + " sfr_t<" + reg_t +"> " + reg.name.toUpperCase() + ";", reg.description));
		}
	}
	
	void output(int len, int pos, int index){
		String ident = TAB+TAB+TAB;
		String rev = "RESERVED" + index + ";";
		String t = len == 4 ? "uint32_t "+rev : (len == 2 ? "uint16_t "+rev : "uint8_t "+rev);
		if(len > 4){
			t = "uint32_t " + "RESERVED" + index + "[" + len/4 + "];";
		}
		writefile(comment(ident + t, String.format(" [%x, %x) ", pos, pos+len)));
	}
	
	void output(Peripheral base, String typeName){
		String ident = TAB+TAB;
		Iterator<Register> it = base.registers.iterator();
		while(it.hasNext()){
			output(it.next(), true);
		}
		writefile(ident + "struct " + typeName.toLowerCase() + "_t{");
		it = base.registers.iterator();
		int pos = 0;
		int index = 0;
		while(it.hasNext()){
			Register reg = it.next();
			int reg_pos = (int)parseLong(reg.addressOffset);
			while(pos < reg_pos){
				if(reg_pos>pos+4){
					int len = reg_pos - pos;
					len = len / 4;
					len = len * 4;
					output(len, pos, index);
					pos += len;
				}else if(reg_pos == pos+4){
					output(4, pos, index);
					pos+=4;
				}else if(reg_pos>=pos+2){
					output(2, pos, index);
					pos+=2;
				}else if(reg_pos>=pos+1){
					output(1, pos, index);
					pos+=4;
				}
				index++;
			}
			output(reg, false);
			pos +=  (int)parseLong(reg.size)/8;
		}
		writefile(ident + "}; //strict " + typeName.toLowerCase() + "_t");
		writefile("");
	}
	
	void output_irqs(Group gp){
		String ident = gp != null ? TAB+TAB : TAB;
		List<Interrupt> c = gp != null ? gp.getIRQs() : getIRQs();
		if(mExtIRQs != null) c = mExtIRQs;
		Iterator<Interrupt> it = c.iterator();
		boolean irq = false;
		while(it.hasNext()){
			Interrupt p = it.next();
			if(p.valid){
				if(!irq ){
					writefile(ident + "enum {");
				}
				irq = true;
				String name = p.name.toUpperCase();
				if(it.hasNext()){
					writefile(comment(ident + TAB + name + "_IRQn = " + p.value + ",", " " + p.value + " " + p.description));
				}else{
					writefile(comment(ident + TAB + name + "_IRQn = " + p.value + "", " " + p.value + " " + p.description));
				}
			}
		}
		if(irq){
			writefile(ident + "};");
		}
	}
	
	void output(Group gp, boolean groupPerFile){
		//log(gp.name);
		
		String ident = TAB+TAB;
		if(groupPerFile){
			output_file_begin(gp);
		}
		output_head(gp, groupPerFile);
		if(groupPerFile){
			output_irqs(gp);
		}
		
		List<Peripheral> c1 = gp.getPeripheralSortByBase();
		if(mExtPers != null) c1 = mExtPers;
		Iterator<Peripheral> i1 = c1.iterator();
		writefile(ident + "enum {");
		while(i1.hasNext()){
			Peripheral p = i1.next();
			String name = p.name.toUpperCase();
			if(i1.hasNext()){
				writefile(ident + TAB + name + "_ADDR = " + p.baseAddress + ",");
			}else{
				writefile(ident + TAB + name + "_ADDR = " + p.baseAddress);
			}
		}
		writefile(ident + "};");
		
		if(!gp.checkMaxPeripheral()){
			// different peripherals in same group, e.g. USB OTG
			Peripheral base = null;
			setCompMethod(Peripheral.REGS);
			// c1 is sort by base address, now sort it by regs
			Collections.sort(c1, Collections.reverseOrder());
			int index = 0;
			List<String> types = new ArrayList<String>();
			List<String> names = new ArrayList<String>();
			for(index =0; index<c1.size(); index++){
				Peripheral t = c1.get(index);
				if(base != null){
					if(base.compareRegs(t)<2) break;
				}
				base = t;
				output(base, base.name);
				types.add(base.name);
				names.add(base.name);
				writefile("");
				writefile("");
			}
			if(index < c1.size()){
				output(base, base.name);
				writefile("");
				writefile("");
				for(;index < c1.size();index++){
					Peripheral t = c1.get(index);
					types.add(base.name);
					names.add(t.name);
				}
			}
			for(index =0;index<names.size();index++){
				String name = names.get(index).toUpperCase();
				writefile(ident + "extern " + types.get(index).toLowerCase() + "_t " + name + ";       SFR_ADDR(" + name+", " + name  + "_ADDR);");
			}
		}else{
			Peripheral base = gp.getMaxPeripheral();
			output(base, base.groupName);
			i1 = c1.iterator();
			while(i1.hasNext()){
				Peripheral p = i1.next();
				String name = p.name.toUpperCase();
				writefile(ident + "extern " + base.groupName.toLowerCase() + "_t " + name + ";       SFR_ADDR(" + name+", " + name  + "_ADDR);");
			}
		}
		
		output_tail(gp, groupPerFile);
		if(groupPerFile){
			output_file_end(gp);
		}
	}
	
	void output(String group, String postfix){
		output(group, postfix, null);
	}
	
	private String mExtInfo = null;
	
	void output(String group, String postfix, String extInfo){
		boolean find = false;
		mExtInfo = extInfo;
		Iterator<Group> it = mGroups.iterator();
		while(it.hasNext()){
			Group gp = it.next();
			if(gp.mName.compareTo(group)==0){
				gp.mPostfix = postfix;
				output(gp, true);
				gp.mPostfix = "";
				find = true;
				break;
			}
		}
		if(find){
			if(extInfo == null)log("Create SFR file for " + group );
		}else{
			log("Peripheral [" + group + "] not exsit");
		}
	}
	
	void output(){
		output(false);
	}
	
	void output(boolean groupPerFile){
		log("output begin:");
		Iterator<Group> it = mGroups.iterator();
		if(groupPerFile){
			log("Create file for each group in " + name + " folder");
			File dir = new File(name.toLowerCase());
			filePrefix = name.toLowerCase() + "/";
			dir.mkdirs();
		}else{
			output_file_begin(null);
			output_irqs(null);
		}
		
		while(it.hasNext()){
			Group gp = it.next(); 
			output(gp, groupPerFile);
		}
		
		if(!groupPerFile){
			output_file_end(null);
		}
	}
	
	String filePrefix = "";
	BufferedWriter fileWriter = null;
	boolean uppercase = false;
	boolean nohpp = false;
	
	void output_file_begin(Group gp){
		String fileName = gp != null ? gp.gpName() : name;
		fileName = filePrefix + (uppercase?fileName.toUpperCase():fileName.toLowerCase()) + (nohpp ? "":".hpp");
		try {
			fileWriter = new BufferedWriter(new FileWriter(fileName));
		} catch (IOException e) {
			log(e.toString());
		}
		writeauthor();
		if(mExtInfo!=null)writefile(mExtInfo); 
		if(gp != null){	
			writefile("#ifndef __SFR_" + gp.gpName().toUpperCase() + "_HPP");
			writefile("#define __SFR_" + gp.gpName().toUpperCase() + "_HPP");
			writefile("");
			writefile("#include <sfr/sfr.hpp>");
			writefile("");
			writefile("namespace sfr{");
		}else{
			log("Create SFR in one file: " + fileName);
			writefile("#ifndef __" + name.toUpperCase() + "_SFR_HPP");
			writefile("#define __" + name.toUpperCase() + "_SFR_HPP");
			writefile("");
			writefile("#include <sfr/sfr.hpp>");
			writefile("");
			writefile("namespace sfr{");
		}
	}
	void output_file_end(Group gp){
		writefile("}");
		writefile("#endif");
		if(fileWriter != null){
			try {
				fileWriter.close();
			} catch (IOException e) {
				log(e.toString());
			}
			fileWriter = null;
		}
	}
	
	void writefile(String str){
		try {
			if(fileWriter != null)fileWriter.write(str + NEWLINE);
		} catch (IOException e) {
			log(e.toString());
		}
	}
	
	void writeauthor(){
		writefile("/**********************************************************************");
		writefile(" * SFR C++ header file for LESTL, generate from " + mFilename);
		writefile(" * For more SVD file, visit here: ");
		writefile(" * http://www.arm.com/zh/products/processors/cortex-m/cortex-microcontroller-software-interface-standard.php");
		writefile(" * Generator author: lxyppc@gmail.com");
		writefile(" **********************************************************************/");
		writefile("");
	}
	
	public static void log(String str){
		System.out.println(str);
	}
}

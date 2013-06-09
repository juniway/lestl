package com.lxyppc;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class SVD_lestl {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		if(args.length<1) usage();
		String filename = args[0];
		boolean upper = false;
		boolean onefile = false;
		boolean nohpp = false;
		List<String> peripherals = new ArrayList<String>();
		int index = 1;
		while(index < args.length){
			String argument = args[index++];
			if("-u".compareTo(argument) == 0){
				upper = true;
			}else if("-s".compareTo(argument) == 0){
				onefile = true;
			}else if("-n".compareTo(argument) == 0){
				nohpp = true;
			}else if("?".compareTo(argument) == 0){
				usage();
			}else if("-?".compareTo(argument) == 0){
				usage();
			}else if("/?".compareTo(argument) == 0){
				usage();
			}else if("-help".compareTo(argument) == 0){
				usage();
			}else if("--help".compareTo(argument) == 0){
				usage();
			}else if("-p".compareTo(argument) == 0){
				if(index == args.length){
					usage("Missing peripheral name after -p");
				}
				peripherals.add(args[index++]);
			}
		}
		
		String[] files = filename.split(",");
		
		if(files.length>1){
			SVDMerge merge = new SVDMerge();
			for(int i=0;i<files.length;i++){
				SVDDevice dev = new SVDDevice(files[i]);
				if(dev.load()){
					dev.uppercase = upper;
					dev.nohpp = nohpp;
					merge.addDevice(dev);
				}
			}
			merge.ouput();
			return;
		}
		
		SVDDevice dev = new SVDDevice(filename);
		if(!dev.load()) return;
		dev.uppercase = upper;
		dev.nohpp = nohpp;
		if(peripherals.size()>0){
			Iterator<String> it = peripherals.iterator();
			while(it.hasNext()){
				dev.output(it.next(),"");
			}
		}else{
			dev.output(!onefile);
		}
	}
	public static void usage(String msg){
		System.out.println(msg);
		usage();
	}
	public static void usage(){
		// 80 cols marker:  01234567890123456789012345678901234567890123456789012345678901234567890123456789
        System.out.println("Usage: SVD_lestl INPUTFILE[,FILE[,FILE...]] [-u | -s | -n | -p Peripheral]");
        System.out.println("     -u       Output file name in upper case");
        System.out.println("     -s       Output all peripheral SFR in one file");
        System.out.println("     -n       No .hpp postfix for SFR file");
        System.out.println("     -p       Output specified peripheral SFR file");
		System.exit(1);
	}

}

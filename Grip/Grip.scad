// ****************************************
// One hand joystick w/ buttons and LEDs
// Filename: Grip.scad
// Created: 4/29/2019
// Revision: 1.0b1 4/29/2019
// Units: mm
// ****************************************
//  ***** Notes *****
//
// ****************************************
//  ***** History *****
//
// ****************************************
//  ***** for STL output *****
// JoystickMountPlate();
// rotate([0,90,0]) Grip();
// rotate([0,-90,0]) GripSide();  // Right Side
// rotate([0,90,0]) mirror([1,0,0]) GripSide(); // Left Side
// ****************************************
//  ***** for Viewing *****
//
// ****************************************
include<CommonStuffSAEmm.scad>

Overlap=0.05;
IDXtra=0.2;
$fn=90;

F1_Space=22;
	F2_Space=22;
	F3_Space=22;
	F4_Space=18;
	AntiSlide_d=4;
	Grip_w=28.5;
	
	Joy_pcb_x=26.2;
	Joy_pcb_y=34.3;
	Joy_pcb_t=1.7;
	MountingHoles_x=20.5;
	MountingHoles_y=26.3;
	MountingHoles_xoff=(Joy_pcb_x-MountingHoles_x)/2;

	module MountingHoles(){
		translate([MountingHoles_xoff,Joy_pcb_y-MountingHoles_xoff,0]) children();
		translate([MountingHoles_xoff+MountingHoles_x,Joy_pcb_y-MountingHoles_xoff,0]) children();
		translate([MountingHoles_xoff,Joy_pcb_y-MountingHoles_xoff-MountingHoles_y,0]) children();
		translate([MountingHoles_xoff+MountingHoles_x,Joy_pcb_y-MountingHoles_xoff-MountingHoles_y,0]) children();
	} // MountingHoles
	
	module RoundRect(X=10,Y=10,Z=1.2,R=3){
		hull(){
			translate([R,R,0]) cylinder(r=R,h=Z);
			translate([X-R,R,0]) cylinder(r=R,h=Z);
			translate([R,Y-R,0]) cylinder(r=R,h=Z);
			translate([X-R,Y-R,0]) cylinder(r=R,h=Z);
		} // hull
	} // RoundRect
	
module JoystickMountPlate(){
	difference(){
		union(){
			translate([0,-3,0]) RoundRect(X=Joy_pcb_x,Y=Joy_pcb_y+6,Z=1.2,R=2);
			RoundRect(X=Joy_pcb_x,Y=Joy_pcb_y,Z=3.2,R=2);
			MountingHoles() cylinder(d=5.5,h=4.2);
		} // union
		translate([0,0,6]) MountingHoles() Bolt4Hole();
		
		// mounting holes
		// ????
	} // diff
} // JoystickMountPlate

//JoystickMountPlate();

module Joystick(){
	
	
	Joy_r=12;
	Joy_h=22;
	Joy_d=22;
	JoyCenter_x=12.5;
	JoyCenter_y=17;
	JoyCenter_z=7.5;
	
	
	
	//MountingHoles() Bolt4Hole();
	
	//difference(){
		translate([0,0,-Joy_pcb_t]) cube([Joy_pcb_x,Joy_pcb_y,Joy_pcb_t]);
	//	MountingHoles() Bolt4Hole();
	//} // diff
	
	translate([JoyCenter_x,JoyCenter_y,JoyCenter_z]) {
		difference(){
			sphere(r=Joy_r);
			translate([0,0,-Joy_r-1]) cylinder(r=Joy_r+1,h=Joy_r+1+3.5);
		} // diff
		cylinder(d=Joy_d-2,h=Joy_h);
		
	}
	
	
} // Joystick

//translate([2,0,5]) rotate([60,0,0]) Joystick();

Handle_Depth=25;

module Grip(){
	
	difference(){
		union(){
			hull(){
			translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([-30,0,0]) translate([0,0,55])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
				
			rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			rotate([-30,0,0]) translate([0,0,45])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			} // hull
			
			hull(){
				translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
				translate([0,-Handle_Depth+AntiSlide_d/2,-F1_Space-F2_Space-F3_Space-F4_Space-10+AntiSlide_d/2])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			} // hull
			
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F1_Space]) cube([Grip_w,Handle_Depth,F1_Space]);
				
				translate([-Overlap,AntiSlide_d*1.6,-F1_Space/2]) rotate([0,90,0]) cylinder(d=F1_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,0,-F1_Space])
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F2_Space]) cube([Grip_w,Handle_Depth,F2_Space]);
				
				translate([-Overlap,AntiSlide_d*1.6,-F2_Space/2]) rotate([0,90,0]) cylinder(d=F2_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space-F2_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,0,-F1_Space-F2_Space])
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F3_Space]) cube([Grip_w,Handle_Depth,F3_Space]);
				
				translate([-Overlap,AntiSlide_d*1.6,-F3_Space/2]) rotate([0,90,0]) cylinder(d=F3_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space-F2_Space-F3_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,0,-F1_Space-F2_Space-F3_Space])
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F4_Space]) cube([Grip_w,Handle_Depth,F4_Space]);
				
				translate([-Overlap,AntiSlide_d*1.5,-F4_Space/2]) rotate([0,90,0]) cylinder(d=F4_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space-F2_Space-F3_Space-F4_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			translate([0,-Handle_Depth+AntiSlide_d/3,-F1_Space-F2_Space-F3_Space-F4_Space-10]) cube([Grip_w,Handle_Depth,10]);
		} // union

		translate([-2,0,0])
		hull(){
			
			translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([-30,0,0]) translate([0,2,2])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([-30,0,0]) translate([0,2,55-2])
			rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
				
			rotate([-30,0,0]) translate([0,-2,2]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			rotate([-30,0,0]) translate([0,-2,45-2])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			} // hull

		// Joystick opening
		translate([0,0,5]) rotate([60,0,0]) Joystick();
		// Joystick back
		translate([0,0,5]) rotate([60,0,0]) translate([-Overlap,0,-10-Joy_pcb_t]) cube([Joy_pcb_x,Joy_pcb_y,10+Joy_pcb_t]);
			
		translate([-Overlap,-Handle_Depth+2,-F1_Space-F2_Space-F3_Space-F4_Space-10+2]) 
			cube([Grip_w-2,Handle_Depth-9,F1_Space+F2_Space+F3_Space+F4_Space+10-1.5]);
			
			// USB connector
		translate([8,-Handle_Depth+4,-F1_Space-F2_Space-F3_Space-F4_Space-10-1]) 
			cube([12.5,11,5]);
			
		// LEDs
		translate([0,-Handle_Depth+4,0]) rotate([-30,0,0]) translate([Grip_w/2,0,51]){
			rotate([90,0,0]) cylinder(d=3,h=10);
			translate([-5,0,0]) rotate([90,0,0]) cylinder(d=3,h=10);
			translate([5,0,0]) rotate([90,0,0]) cylinder(d=3,h=10);
		}
	} // diff
} // Grip

//rotate([0,90,0]) Grip();

module GripSide(){
	Grip_w=2;
	
	difference(){
		union(){
		hull(){
			translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			translate([Grip_w,-Handle_Depth+AntiSlide_d/2,0]) sphere(d=AntiSlide_d);
				
			translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([-30,0,0]) translate([0,0,55])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			translate([Grip_w,-Handle_Depth+AntiSlide_d/2,0]) rotate([-30,0,0]) translate([0,0,55]) sphere(d=AntiSlide_d);
				
			rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			translate([Grip_w,0,0]) sphere(d=AntiSlide_d);
				
			rotate([-30,0,0]) translate([0,0,45])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			translate([Grip_w,0,0]) rotate([-30,0,0]) translate([0,0,45]) sphere(d=AntiSlide_d);
		} // hull
			
			hull(){
				translate([0,-Handle_Depth+AntiSlide_d/2,0]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
				translate([0,-Handle_Depth+AntiSlide_d/2,-F1_Space-F2_Space-F3_Space-F4_Space-10+AntiSlide_d/2])
				rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			} // hull
			
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F1_Space]) cube([Grip_w,Handle_Depth,F1_Space]);
				
				translate([-Overlap,AntiSlide_d*1.6,-F1_Space/2]) rotate([0,90,0]) cylinder(d=F1_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,0,-F1_Space])
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F2_Space]) cube([Grip_w,Handle_Depth,F2_Space]);
				
				translate([-Overlap,AntiSlide_d*1.6,-F2_Space/2]) rotate([0,90,0]) cylinder(d=F2_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space-F2_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,0,-F1_Space-F2_Space])
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F3_Space]) cube([Grip_w,Handle_Depth,F3_Space]);
				
				translate([-Overlap,AntiSlide_d*1.6,-F3_Space/2]) rotate([0,90,0]) cylinder(d=F3_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space-F2_Space-F3_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			
			translate([0,0,-F1_Space-F2_Space-F3_Space])
			difference(){
				translate([0,-Handle_Depth+AntiSlide_d/3,-F4_Space]) cube([Grip_w,Handle_Depth,F4_Space]);
				
				translate([-Overlap,AntiSlide_d*1.5,-F4_Space/2]) rotate([0,90,0]) cylinder(d=F4_Space,h=Grip_w+Overlap*2);
			} // diff
			
			translate([0,0,-F1_Space-F2_Space-F3_Space-F4_Space]) rotate([0,90,0]) cylinder(d=AntiSlide_d,h=Grip_w);
			translate([0,-Handle_Depth+AntiSlide_d/3,-F1_Space-F2_Space-F3_Space-F4_Space-10]) cube([Grip_w,Handle_Depth,10]);
		

		

		
	
	
	translate([Grip_w,0,0]) sphere(d=AntiSlide_d);
	translate([Grip_w,0,-F1_Space]) sphere(d=AntiSlide_d);
	translate([Grip_w,0,-F1_Space-F2_Space]) sphere(d=AntiSlide_d);
	translate([Grip_w,0,-F1_Space-F2_Space-F3_Space]) sphere(d=AntiSlide_d);
			
	hull(){
		translate([Grip_w,-Handle_Depth+AntiSlide_d/2,-F1_Space-F2_Space-F3_Space-F4_Space]) sphere(d=AntiSlide_d);
		translate([Grip_w,-Handle_Depth+AntiSlide_d/2,0]) sphere(d=AntiSlide_d);
		
		translate([Grip_w,-6.5,-F1_Space-F2_Space-F3_Space-F4_Space]) sphere(d=AntiSlide_d);
		translate([Grip_w,-6.5,0]) sphere(d=AntiSlide_d);
	}
			
	hull(){
		translate([Grip_w,0,-F1_Space-F2_Space-F3_Space-F4_Space]) sphere(d=AntiSlide_d);
		translate([Grip_w,0,-F1_Space-F2_Space-F3_Space-F4_Space-10+AntiSlide_d/2]) sphere(d=AntiSlide_d);
		translate([Grip_w,-Handle_Depth+AntiSlide_d/2,-F1_Space-F2_Space-F3_Space-F4_Space-10+AntiSlide_d/2]) sphere(d=AntiSlide_d);
		translate([Grip_w,-Handle_Depth+AntiSlide_d/2,-F1_Space-F2_Space-F3_Space-F4_Space]) sphere(d=AntiSlide_d);
	} // hull
			
	//hull(){
	//translate([Grip_w,-18,-50]) cylinder(d=6,h=50);
	//translate([Grip_w,-5,-50]) cylinder(d=6,h=50);
	//}
	
	translate([Grip_w,0,0])
		translate([0,F1_Space/2-4.1,-F1_Space/2])
			rotate([0,90,0]) rotate([0,0,-149]) 
				rotate_extrude(angle = 117, convexity = 2) translate([F1_Space/2+AntiSlide_d/1.7,0]) circle(d=AntiSlide_d);
				
	translate([Grip_w,0,-F1_Space])
		translate([0,F1_Space/2-4.1,-F1_Space/2])
			rotate([0,90,0]) rotate([0,0,-149]) 
				rotate_extrude(angle = 117, convexity = 2) translate([F1_Space/2+AntiSlide_d/1.7,0]) circle(d=AntiSlide_d);
				
	translate([Grip_w,0,-F1_Space-F2_Space])
		translate([0,F1_Space/2-4.1,-F1_Space/2])
			rotate([0,90,0]) rotate([0,0,-149]) 
				rotate_extrude(angle = 117, convexity = 2) translate([F1_Space/2+AntiSlide_d/1.7,0]) circle(d=AntiSlide_d);
	
		translate([Grip_w,0,-F1_Space-F2_Space-F3_Space])
		translate([0,F1_Space/2-4.5,-F4_Space/2])
			rotate([0,90,0]) rotate([0,0,-140]) 
				rotate_extrude(angle = 103, convexity = 2) translate([F4_Space/2+AntiSlide_d/1.7,0]) circle(d=AntiSlide_d);
			} // union
			
			
			translate([4,5.5,44]) rotate([0,90,0]) Bolt4FlatHeadHole(depth=8,lAccess=12);
			
			translate([4,-20,0]) rotate([0,90,0]) Bolt4FlatHeadHole(depth=8,lAccess=12);
			translate([4,-4,0]) rotate([0,90,0]) Bolt4FlatHeadHole(depth=8,lAccess=12);
			
			translate([4,-4,-F1_Space-F2_Space-F3_Space-F3_Space]) rotate([0,90,0]) Bolt4FlatHeadHole(depth=8,lAccess=12);
			
		} // diff
		
} // GripSide

//rotate([0,-90,0]) GripSide();













tol = 0.035;

mount_h = 6;
mount_r_in = 0.92;
mount_r_out = 3;
$fn = 20;
mounts_positions = [[5, 10, 0], [5, 40, 0], [35, 50, 0], [70, 50, 0], [70, 4, 0], [100, 5, 0]];
module mount()
    difference()
    {
        cylinder(mount_h, mount_r_out - tol, mount_r_out - tol, [0,0,mount_h/2]);
        cylinder(mount_h, mount_r_in + tol, mount_r_in + tol, [0,0,mount_h/2]);
    }    


d = 2;
board_margin = 1;
pcb_w = 105;
pcb_h = 65;
pcb_thickness = 1.6;
w = pcb_w + 2 * board_margin + d ;
h = pcb_h + 2 * board_margin + d ;
height = 44;
module mounts()
    for (position = mounts_positions)
        translate(position)
            mount();

motor_drill_h = 13;
motor_drill_w = 32;
motor_drill_xs = [16.793, 52.335, 87.913];
module motor_drills(){
    for (x = motor_drill_xs)
        translate([x, pcb_h + board_margin + d/2, mount_h + tol + pcb_thickness + motor_drill_h/2])
            cube([motor_drill_w, 10, motor_drill_h], true);
}

usb_drill_h = 6;
usb_drill_w = 19;
usb_drill_y = 10.32;
module usb_drill(){
    translate([-board_margin - d/2, usb_drill_y, mount_h + tol + pcb_thickness + 10 + height/2])
        cube([5, usb_drill_w + 2 * tol, height + 2 * tol], true);
}

dc_drill_d = 8;
dc_drill_y = 57;
dc_drill_z = mount_h + tol + pcb_thickness + 19;
module dc_drill(){
    translate([-board_margin - d/2, dc_drill_y, dc_drill_z])
        rotate(90, [0,1,0])
            cylinder(5, d=dc_drill_d + 2 * tol, center=true);
}

module label_drills(){
    translate([w - 18.2, h + d + 2 * board_margin - 0.5, 23])
        rotate(90,[1,0,0])
            scale([-1,1,1])
                linear_extrude(height = 10, center = true, convexity = 5)
                    text("0          1          2", size = 7.8, font = "Liberation Sans:style=Bold");
}

thermal_drill_r = 1.8;
thermal_drill_grid = 6;
module thermal_drill(thermal_drill_w){
    rotate(45, [0,0,1])
    for( x = [0: thermal_drill_grid: thermal_drill_w / sqrt(2) - thermal_drill_r])
        union(){
            translate([x, 0, 0])
                cube([2 * thermal_drill_r, 2 * (thermal_drill_w / sqrt(2) - thermal_drill_r - x), 5], center=true);
            translate([-x, 0, 0])
                cube([2 * thermal_drill_r, 2 * (thermal_drill_w / sqrt(2) - thermal_drill_r - x), 5], center=true);
        }
}

thermal_drill_z = mount_h + tol + pcb_thickness + 13;
module thermal_drills(){
    union(){
        translate([-board_margin - d/2, 36, thermal_drill_z])
            rotate(90, [0,1,0])
                thermal_drill(25);
        translate([pcb_w / 2, -board_margin - d/2, thermal_drill_z])
            rotate(90, [1,0,0])
                thermal_drill(25);
        translate([pcb_w / 2 - 32, -board_margin - d/2, thermal_drill_z])
            rotate(90, [1,0,0])
                thermal_drill(25);
        translate([pcb_w / 2 + 32, -board_margin - d/2, thermal_drill_z])
            rotate(90, [1,0,0])
                thermal_drill(25);
        translate([pcb_w + board_margin + d/2, pcb_h / 2 - 15, thermal_drill_z])
            rotate(90, [0,1,0])
                thermal_drill(25);
        translate([pcb_w + board_margin + d/2, pcb_h / 2 + 15, thermal_drill_z])
            rotate(90, [0,1,0])
                thermal_drill(25);
    }
}

top_height = 8;
top_groove = 5;
top_usb_h = height - (mount_h + tol + pcb_thickness + 10) - usb_drill_h;
module top(){
    union(){
        translate([-board_margin - d/2, -board_margin - d/2, 0])
            difference(){
                translate([-d/2, -d/2, -d/2])
                    cube([w + d, h + d, top_height]);
                translate([d/2, d/2, d/2])
                    cube([w - d, h - d, top_height]);
            for(x = [7.5 + d/2 + board_margin:10:w-7.5])
                translate([x, h / 2, top_height - top_groove / 2])
                    cube([top_groove + 2 * tol, h + 10, top_groove + 2 * tol], center=true);
            for(y = [7.5 + d/2 + board_margin:10:h-7.5])
                translate([w, y, top_height - top_groove / 2])
                    cube([10, top_groove + 2 * tol, top_groove + 2 * tol], center=true);
            for(y = [27.5 + d/2 + board_margin:10:h-7.5])
                translate([0, y, top_height - top_groove / 2])
                    cube([10, top_groove + 2 * tol, top_groove + 2 * tol], center=true);
            //translate([w/2 + 34, h/2 - 17, 0])
            //    thermal_drill(29);
            //translate([w/2 + 34, h/2 + 17, 0])
            //    thermal_drill(29);
            //translate([w/2, h/2 - 17, 0])
            //    thermal_drill(29);
            //translate([w/2, h/2 + 17, 0])
            //    thermal_drill(29);
            //translate([w/2 - 34, h/2 - 17, 0])
            //    thermal_drill(29);
            //translate([w/2 - 34, h/2 + 17, 0])
            //    thermal_drill(29);
            }
        translate([-board_margin - d/2, usb_drill_y, top_usb_h / 2])
            cube([d, usb_drill_w - 2 * tol, top_usb_h - 2 * tol], true);
    }
}

table_mount_h = 25;
table_mount_d = 7;
table_mount_dist = 15;
table_mount_raster = 25.4;
table_mount_support = 7.6;
module table_mount(){
    union(){
        difference(){
            linear_extrude(height = d, center = true, convexity = 5)
                polygon([[-d/2, 0], [table_mount_h - d/2, table_mount_h], 
                        [w - table_mount_h + d/2, table_mount_h], [w + d/2, 0]]);
            for(i = [-1,0,1])
                translate([w/2 + table_mount_raster * i, table_mount_dist, 0])
                    cylinder(5, d=table_mount_d + 2 * tol, center=true);
        }
        for(i = [-1.5, -0.5, 0.5, 1.5])
            translate([w/2 + table_mount_raster * i, 0, 0])
                rotate(-90, [0,1,0])
                    linear_extrude(height = d, center = true, convexity = 5)
                        polygon([[0,0], [table_mount_support, 0], [0, table_mount_support]]);
    }
}

module bottom(){
    difference(){
        union(){
            mounts();
            translate([-board_margin - d/2, h - board_margin, 0])
                table_mount();
            //translate([-board_margin - d/2 + w, - board_margin - d, 0])
            //    rotate(180,[0,0,1])
            //        table_mount();
            translate([-board_margin - d/2, -board_margin - d/2, 0])
                difference(){
                    translate([-d/2, -d/2, -d/2])
                        cube([w + d, h + d, height]);
                    translate([d/2, d/2, d/2])
                        cube([w - d, h - d, height]);
            }
        }
        motor_drills();
        label_drills();
        usb_drill();
        dc_drill();
        thermal_drills();
        translate([0,0,height])
            scale([1,1,-1])
                minkowski(){
                    cube(4*tol, center=true);
                    top();
                }
   }
}

bottom();
//translate([0,0,height])
    //scale([1,1,-1])
       //top();
translate([0,-15,0])
    scale([1,-1,1])
        top();

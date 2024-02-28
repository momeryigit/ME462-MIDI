
/*
Author: Lukas M. Süss aka mechadense
Date: 2017-01-12 ... 2018-05
License: Public Domain
Title: Supportless bellow for TPU filament printing
*/


// TODO maybe:
// resolve BULKY SIZE ISSUE
// maybe add optional stiffening sprockets at the ends ??
// maybe make it thingiverse customizer ready?
// scale the support wallthickness with the main diameter?

// Testprint turned out pretty stiff => thinner walls?


nseg = 4; // how many zig-zag segments

dout = 25; // outer diameter of the bellow
  rout = dout/2;

hseg = 3.0; // half zig-zag height


eps = 0.05*1;
$fa = 4; // triangulation resolution
$fs = 0.3; // triangulation resolution

twall = 1.5; // wall thicknes of the bellows zig-zag section
twall2 = twall*sqrt(2); // horizontal thickness



dtor=3; // mounting torus cut diameter
  rtor = dtor/2;

dhole = 5; // diameter of air supply hole
tholewall = 3; // wall thickness around air supply hole


//tsupportwall = dtor/2; // no effect ???
tsupportwall = dtor/2+twall*1.5; // fixed value is not good ...




debug_cut = "yes"; // [no,yes]


// #### indirectly determined sizes

hsupport = rout; // total height of endpieces

din = dout - 2*hseg - 2*twall2; rin= din/2; // inner bellow diameter
echo("inner bellow diameter is: ",din);

ltotal = hsupport+nseg*2*hseg+hsupport;


// #####################################


difference()
{
  fullbellow();
  if(debug_cut=="yes")
  {
    translate([0,0,-1])
    cube([dout,dout,ltotal+2]);
  }
}

module fullbellow ()
{

  echo("total length of bellow is:",ltotal);

  cap(0);

  translate([0,0,hseg+hsupport]) 
  for(i=[0:nseg-1])
  {
    translate([0,0,i*2*hseg])
    necksegment();
  }

  translate([0,0,ltotal])
  scale([1,1,-1]) cap();
}




module necksegment()
{
  segment();
  scale([1,1,-1]) segment();
}


module segment()
{
  difference()
  {
    cylinder(r1=rin+twall2,r2=rout,h=hseg);
    translate([0,0,-eps])
    cylinder(r1=rin,r2=rout-twall2,h=hseg+2*eps);
  }
}



module cap(dhole_ifset = dhole)
{
  rhole=dhole/2;
  rhole_ifset=dhole_ifset/2;
  
  // note: thickness of conical wall shrinks down to twall2

  difference()
  {
    // full base body
    union()
    {
      cylinder(r1=twall2,r2=rout,h=rout);
      support();
    }

    // hollow out internal airspace (conical)
    translate([0,0,tsupportwall*1])
      cylinder(r1=eps,r2=rout-twall2,h=rout-tsupportwall*1+1*eps);
    // this is bending the 45° overhang rule a bit ... 
    //    ... => maybe elongate endpieces a bit instead
    //           to guatŕantee airtightess

    // cut hole if desired
    if(dhole > 0)
    {
      color("red")
      translate([0,0,-2])
        cylinder(r=dhole_ifset/2,h=rout+2);
    }
  }

  module support()
  {
    // holewall
    cylinder(r=rhole+tholewall,h=hsupport);
    // support ring
    difference()
    { // outer base body
      cylinder(r=rout,h=hsupport);
      // main cut
      translate([0,0,-1])
        cylinder(r=rout-tsupportwall,h=hsupport+2);
      // mounting torus cut
      translate([0,0,rtor+dtor])
      rotate_extrude(convexity = 3)
        translate([rout,0,0]) circle(r=rtor,$fn=4);
    }

  }
}
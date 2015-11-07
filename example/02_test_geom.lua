--====================================================================
local ffi = require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds  = require "drawstuff"
--====================================================================

local radius = 0.25;
local length = 1.0;
local sides  = ffi.new("double[3]")
sides[0]=0.5
sides[1]=0.5
sides[2]=1.0

local world = nil;-- world
local ball  = nil;-- body
local mass  = 1.0;  --(kg)

local DENSITY  = (5.0)

--body
local sphere    = { body=nil }
local box       = { body=nil }
local capsule   = { body=nil }
local cylinder = { body=nil }

--====================================================================
function start()
    --camera
    local xyz = ffi.new("float[3]");
    xyz[0] =  0.0;
    xyz[1] = -3.0;
    xyz[2] =  1.0;
    local hpr = ffi.new("float[3]");
    hpr[0] = 90.0
    hpr[1] = 0.0
    hpr[2] = 0.0
    --================================================================
    ds.dsSetViewpoint (xyz,hpr);
end
--====================================================================
function simLoop (pause)

    --world step
    ode.dWorldStep(world,0.05);

    -- spere
    ds.dsSetColor(1,0,0);
    ds.dsSetSphereQuality(3);
    local pos1 = ode.dBodyGetPosition(sphere.body);
    local R1   = ode.dBodyGetRotation(sphere.body);
    ds.dsDrawSphereD(pos1,R1,radius);

    -- Cylinder
    ds.dsSetColorAlpha (0,1,0,1);
    local pos2 = ode.dBodyGetPosition(cylinder.body);
    local R2   = ode.dBodyGetRotation(cylinder.body);
    ds.dsDrawCylinderD(pos2,R2,length,radius);

    -- Capsule
    ds.dsSetColorAlpha (1,1,1,1);
    local pos3 = ode.dBodyGetPosition(capsule.body);
    local R3   = ode.dBodyGetRotation(capsule.body);
    ds.dsDrawCapsuleD(pos3,R3,length,radius);

    -- Box
    ds.dsSetColorAlpha (0,0,1,1);
    local pos4 = ode.dBodyGetPosition(box.body);
    local R4   = ode.dBodyGetRotation(box.body);
    ds.dsDrawBoxD(pos4,R4,sides);

    -- Line
    local posA = ffi.new("double[3]")
    posA[0] = 0;
    posA[1] = 5;
    posA[2] = 0;
    local posB=ffi.new("double[3]")
    posB[0] = 0;
    posB[1] = 5;
    posB[2] = 1.9;
    ds.dsDrawLineD(posA,posB);
end
--====================================================================
function main ()

    --================================================================
    ode.dInitODE();
    world = ode.dWorldCreate();
    ode.dWorldSetGravity(world,0,0,-0.001);

    --ball position
    local x0 = 0.0;
    local y0 = 0.0;
    local z0 = 1.0;
    local m = ffi.new("dMass[1]");

    ode.dMassSetZero (m);
    --sphere
    sphere.body = ode.dBodyCreate (world);
    local radius = 0.5;
    ode.dMassSetSphere (m,DENSITY,radius);
    ode.dBodySetMass (sphere.body,m);
    ode.dBodySetPosition (sphere.body,0,1, 1);

    --box
    box.body = ode.dBodyCreate (world);
    ode.dMassSetBox (m,DENSITY,sides[0],sides[1],sides[2]);
    ode.dBodySetMass (box.body,m);
    ode.dBodySetPosition (box.body,0,2,1);

    --Capsule
    capsule.body = ode.dBodyCreate (world);
    ode.dMassSetCapsule(m,DENSITY,3,radius,length);
    ode.dBodySetMass (capsule.body,m);
    ode.dBodySetPosition (capsule.body,0,4,1);

    --Cylinder
    cylinder.body = ode.dBodyCreate (world);
    ode.dMassSetCylinder(m,DENSITY,3,radius,length);
    ode.dBodySetMass (cylinder.body,m);
    ode.dBodySetPosition (cylinder.body,0,3,1);

    -- drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].start = nil;
    func[0].step = simLoop;
    func[0].command = nil;      -- nothing func so nil
    func[0].stop  = nil;        -- nothing func so nil
    func[0].path_to_textures = "./textures";

    --drawstuff
    local argv =ffi.new("char*[1]");
    ds.dsSimulationLoop (0,nil,352,288,func[0]);

    -- destroy world
    ode.dWorldDestroy (world);
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================

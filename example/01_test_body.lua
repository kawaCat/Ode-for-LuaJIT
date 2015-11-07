--====================================================================
local ffi = require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds  = require "drawstuff"
--====================================================================
local world  = nil; -- world
local ball   = nil; -- body
local radius = 0.2;
local mass   = 1.0; --(kg)

-- drawstuff simlation loop func
--====================================================================
function simLoop (pause)

    --world step
    ode.dWorldStep(world,0.05); --world step

    --
    local pos = ode.dBodyGetPosition(ball); --get Pos
    local R = ode.dBodyGetRotation(ball); --get Rot

    --color
    ds.dsSetColor(1.0,0.0,0.0);
    --================================================================

    --ds.dsDrawSphere(pos,R,radius); -- in case of using single ode.
    --================================================================
    ds.dsDrawSphereD(pos,R,radius);  -- in case of using double ode.

end
--====================================================================

-- drawstuff start func
--====================================================================
function start()
    local xyz = ffi.new("float[3]");
    xyz[0] =  0.0; -- x
    xyz[1] = -3.0; -- y
    xyz[2] =  1.0; -- z
    local hpr = ffi.new("float[3]");
    hpr[0] = 90.0;
    hpr[1] = 0.0;
    hpr[2] = 0.0;
    --================================================================
    ds.dsSetViewpoint (xyz,hpr); --set camera pos (set view)
end
--====================================================================

--====================================================================
function main ()

    --================================================================
    ode.dInitODE();
    world = ode.dWorldCreate();
    ode.dWorldSetGravity(world,0,0,-0.001);

    -- ball position
    local x0 = 0.0;
    local y0 = 0.0;
    local z0 = 1.0;
    local m1 = ffi.new("dMass[1]"); --

    ball = ode.dBodyCreate(world);
    ode.dMassSetZero(m1);
    ode.dMassSetSphereTotal(m1,mass,radius);
    ode.dBodySetMass(ball,m1);
    ode.dBodySetPosition(ball, x0, y0, z0);

    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = nil;      -- nothing func so nil
    func[0].stop  = nil;        -- nothing func so nil
    func[0].path_to_textures = "./textures";

    --drawstuff
    local argv =ffi.new("char*[1]");
    ds.dsSimulationLoop (0,argv,352,288,func[0]);

    -- destroy world
    ode.dWorldDestroy (world);
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================

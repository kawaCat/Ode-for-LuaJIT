
--====================================================================
local ffi =require"ffi"
--====================================================================
local ds =require "drawstuff"
local ode = require "ode_ffi"
--====================================================================
local world = nil;-- world
local space  =nil --space;
local ground  =nil --space;
local contactgroup =nil;
local flag = 0;


local joint =nil --
--====================================================================
local ball =
{
      body = nil -- body
    , geom = nil -- geom
    , radius =0
    , length =0
    , mass    =0
};
--====================================================================
local pole =
{
      body = nil -- body
    , geom = nil -- geom
    , radius=0
    , length=0
    , mass=0
};
--====================================================================

-- contact callback func
--====================================================================
function nearCallback(data,  o1, o2)
    local N = 10;
    local contact =ffi.new("dContact[?]",N );
    local isGround = ((ground == o1) or (ground == o2)) or false;
    local n =  ode.dCollide(o1,o2,N,contact[0].geom,ffi.sizeof("dContact"));
    --================================================================
    if (isGround)
    then
        --============================================================
        for i = 0,n-1
        do
            contact[i].surface.mode = ode.dContactBounce;
            contact[i].surface.mu   = (1.0/0.0) -- inf(dInfinity);
            contact[i].surface.bounce     = 1.0; -- (0.0~1.0) restitution parameter
            contact[i].surface.bounce_vel = 0.0; -- minimum incoming velocity for bounce
            local c = ode.dJointCreateContact(world,contactgroup,contact[i]);
            ode.dJointAttach( c
                            , ode.dGeomGetBody(contact[i].geom.g1 )
                            , ode.dGeomGetBody(contact[i].geom.g2 ) );
        end
        --============================================================
    end
end
-- drawstuff simlation loop func
--====================================================================
function simLoop (pause)

    --check Contact
    local callBack = ffi.cast ( "dNearCallback",nearCallback)
    ode.dSpaceCollide(space,nil,callBack);
    callBack:free();

    --world step
    ode.dWorldStep(world,0.3);
    ode.dJointGroupEmpty(contactgroup);
    --

    --================================================================
    local pos = ode.dBodyGetPosition(ball.body);
    local R = ode.dBodyGetRotation(ball.body);
    ds.dsSetColor(1.0,0.0,0.0);
    --ds.dsDrawSphere(pos,R,radius); -- in case of using single ode.
    --================================================================
    ds.dsDrawSphereD(pos,R,ball.radius);  -- in case of using double ode.

    -- draw a ccylinder
    local pos2 = ode.dBodyGetPosition(pole.body);
    local R2   = ode.dBodyGetRotation(pole.body);
    -- ds.dsDrawCapsule(pos2,R2,pole.length,pole.radius);
    --================================================================
    ds.dsDrawCapsuleD(pos2,R2,pole.length,pole.radius);
end
--====================================================================

-- drawstuff start func
--====================================================================
function start()
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

--====================================================================
function createBallAndPole()
    -- ball Position
    local x0 = 0.0;
    local y0 = 0.0;
    local z0 = 1.0;
    local m1 = ffi.new("dMass[1]");
    ball.radius = 0.2;
    ball.mass   = 1.0;
    ball.body = ode.dBodyCreate(world);
    ode.dMassSetZero(m1);
    ode.dMassSetSphereTotal(m1,ball.mass,ball.radius);
    ode.dBodySetMass(ball.body,m1);
    ode.dBodySetPosition(ball.body, x0, y0, z0);

    ball.geom = ode.dCreateSphere(space,ball.radius);
    ode.dGeomSetBody(ball.geom,ball.body);

    -- pole
    pole.radius = 0.025;
    pole.length = 1.0;
    pole.mass   = 1.0;
    pole.body = ode.dBodyCreate(world);
    ode.dMassSetZero(m1);
    ode.dMassSetCapsule(m1,pole.mass,3,pole.radius,pole.length);
    ode.dBodySetMass(pole.body,m1);
    ode.dBodySetPosition(pole.body, x0, y0, z0 - 0.5 * pole.length);

    pole.geom = ode.dCreateCapsule(space,pole.radius,pole.length);
    ode.dGeomSetBody(pole.geom,pole.body);

    -- hinge joint
    joint = ode.dJointCreateHinge(world, nil);
    ode.dJointAttach(joint, ball.body,pole.body);
    ode.dJointSetHingeAnchor(joint, x0, y0, z0 - ball.radius);
    ode.dJointSetHingeAxis(joint, 1, 0, 0);
end
--====================================================================

--====================================================================
function main ()
    --================================================================
    ode.dInitODE();
    world = ode.dWorldCreate();
    ode.dWorldSetGravity(world,0,0,-0.001);

    space = ode.dHashSpaceCreate(nil);
    contactgroup = ode.dJointGroupCreate(0);
    ground = ode.dCreatePlane(space,0,0,1,0);

    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = nil;      -- nothing func so nil
    func[0].stop  = nil;        -- nothing func so nil
    func[0].path_to_textures = "./textures";

    createBallAndPole();

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

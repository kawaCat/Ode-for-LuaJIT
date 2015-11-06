
--====================================================================
local ffi =require"ffi"
--====================================================================
local ode = require "ode_ffi"
local ds =require "drawstuff"
--====================================================================

--====================================================================
local levels = 3;
local ncards = 0;
--====================================================================
local space = nil;
local world = nil;
local contactgroup = nil;
--====================================================================
local cwidth=.5 
local cthikness=.02
local clength=1;
local cards ={};

--====================================================================
function createCard()
    local Card ={}
    Card.body =nil;
    Card.geom=nil
    Card.sides = ffi.new("dReal[3]",cwidth,cthikness,clength);
    --================================================================
    function Card:init()
        self.body = ode.dBodyCreate(world);
        self.geom = ode.dCreateBox(space, self.sides[0], self.sides[1], self.sides[2]);
        ode.dGeomSetBody(self.geom, self.body);
        --ode.dGeomSetData(self.geom, self);
        local  mass = ffi.new("dMass[1]");
        ode.dMassSetBox(mass,1, self.sides[0], self.sides[1], self.sides[2]);
        ode.dBodySetMass(self.body, mass);
    end 
    --================================================================
    function Card:destroy()
        ode.dGeomDestroy(self.geom);
        ode.dBodyDestroy(self.body);
        ode.dSpaceSetCleanup (space,0);
    end
    --================================================================
    function Card:draw()
        ds.dsDrawBoxD(ode.dBodyGetPosition(self.body),
                      ode.dBodyGetRotation(self.body), self.sides);
    end
    --================================================================
    Card:init()
    --================================================================
    return Card;
end 
--====================================================================

--====================================================================
function getNCards(levels)
    return ((3*levels*levels) + levels) / 2;
end
--====================================================================

--====================================================================
function place_cards()
    
    ncards = getNCards(levels);
    
    -- destroy removed cards
    --================================================================
    for i, v  in ipairs(cards)
    do
        v:destroy();
    end 
    cards ={};
    --================================================================

    -- construct new cards 
    --================================================================
    for i=0,ncards-1
    do
        cards[i] = createCard()
    end 
    --================================================================
    
    -- for each level
    local M_PI = math.pi;
    local c = 0;
    local right = ffi.new("dMatrix3[1]")
    local left  = ffi.new("dMatrix3[1]")
    local hrot  = ffi.new("dMatrix3[1]")
    local angle =20*M_PI/180.0;
    local angle2 = 91*M_PI/180.0;
    --================================================================
    ode.dRFromAxisAndAngle(right[0],  1, 0, 0, -angle);
    ode.dRFromAxisAndAngle(left[0] ,  1, 0, 0,  angle);
    ode.dRFromAxisAndAngle(hrot[0] ,  1, 0,0, angle2);
    --================================================================
    local eps = 0.05;
    local vstep = math.cos(angle)*clength + eps;
    local hstep = math.sin(angle)*clength + eps;
    --================================================================
    for lvl=0,levels
    do
        -- there are 3*(levels-lvl)-1 cards in each level, except last
        local n = (levels-lvl);
        local height = (lvl)*vstep + vstep/2;
        -- inclined cards
        for i=0, (2*n)-1,1
        do
            ode.dBodySetPosition( cards[c].body
                                , 0
                                , -n*hstep + hstep*i
                                , height
                                );
            --========================================================
            if (i%2 ~=0)
            then 
                ode.dBodySetRotation(cards[c].body, left[0]);
            else
                ode.dBodySetRotation(cards[c].body, right[0]);
            end 
            --========================================================
            c =c+1
        end
        --============================================================
        -- top of the house
        if (n==1)  
        then
            break;
        end 
        --============================================================
        -- horizontal cards
        for i=0, n-2
        do 
            ode.dBodySetPosition( cards[c].body
                                , 0
                                , -(n-1 - (clength-hstep)/2)*hstep + 2*hstep*i
                                , height + vstep/2);
            ode.dBodySetRotation(cards[c].body, hrot[0]);
            c =c+1
        end 
        --============================================================
    end
end
--====================================================================


--====================================================================
function nearCallback (data, o1, o2)
    -- exit without doing anything if the two bodies are connected by a joint
    local b1 = ode.dGeomGetBody(o1);
    local b2 = ode.dGeomGetBody(o2);
    --================================================================
    local MAX_CONTACTS = 8;
    local contact = ffi.new("dContact[?]",MAX_CONTACTS);
    --================================================================
    local numc = ode.dCollide ( o1
                              , o2
                              , MAX_CONTACTS
                              , contact[0].geom
                              , ffi.sizeof("dContact"));
    --================================================================
    for i=0,numc-1
    do
        contact[i].surface.mode = ode.dContactApprox1;
        contact[i].surface.mu = 5;
        local  c = ode.dJointCreateContact (world, contactgroup, contact+i);
        ode.dJointAttach (c, b1, b2);
    end
    --================================================================
end   
--====================================================================

--====================================================================
-- determines whether a space
-- from.
-- http://so-zou.jp/robot/tech/physics-engine/ode/collision-detection/sample-code.htm
--====================================================================
function nearCallBack_checkSpace(data,o1,o2)
    
    -- nice google translate.
    -- o1, one of o2 determines whether a space .
    if( ode.dGeomIsSpace( o1 ) or  ode.dGeomIsSpace( o2 ) )
    then 
        -- o1, o2 also in consideration is a different space , and
        -- performs collision detection.
        local callBack = ffi.cast ( "dNearCallback",nearCallback)
        ode.dSpaceCollide2( o1, o2, data, callBack);
        --============================================================
        if( ode.dGeomIsSpace( o1 ) ==true )
        then
            -- if o1 is space , and
            -- the collision detection to target all of the geometry contained therein
            ode.dSpaceCollide( o1, data, callBack );
        end
        --============================================================
        if( ode.dGeomIsSpace( o2 )==true )
        then
            -- if o2 is space , and
            -- the collision detection to target all of the geometry contained therein
            ode.dSpaceCollide( o2, data, callBack );
        end 
        --============================================================
        callBack:free(); -- free luajit ffi callback.
    else
        nearCallback (data, o1, o2);
    end 
end 
--====================================================================

-- drawstuff simlation loop func
--====================================================================
function simLoop (pause)
    --================================================================
    if (pause == 0 ) -- pause==1 : pause
    then 
        --============================================================
        local callBack = ffi.cast ( "dNearCallback",nearCallBack_checkSpace)
        ode.dSpaceCollide(space,nil,callBack);
        callBack:free();
        --============================================================
        ode.dWorldQuickStep (world,0.01);
         
        ode.dJointGroupEmpty (contactgroup);
        --============================================================
    end
    --================================================================
    for i,v in ipairs(cards)
    do
        ds.dsSetColor (1, i/ncards, 0);
        v:draw();
    end 
    --================================================================
end
--====================================================================

-- drawstuff start func
--====================================================================
function start()
    print ([[
    Controls:
       SPACE - reposition cards
       -     - one less level
       =     - one more level
    ]]);
end
--====================================================================

--====================================================================
-- called when a key pressed
function command (cmd)
    --================================================================
    if (   cmd == string.byte'=' )
    then 
        levels = levels +1;
        place_cards();
        --============================================================
    elseif (   cmd == string.byte'-' )
    then 
        if ( levels > 1)
        then 
            levels =levels -1;
        end 
        place_cards();
        --============================================================
    elseif (cmd == string.byte' ')
    then 
        place_cards();
    end
    --================================================================
end 
--====================================================================

--====================================================================
function main ()
    --================================================================
    ode.dInitODE();
    --================================================================
    --drawstuff function struct
    local func = ffi.new("dsFunctions[1]");
    func[0].version = ds.DS_VERSION;
    func[0].start = start;
    func[0].step = simLoop;
    func[0].command = command;     
    func[0].stop  = nil;       
    func[0].path_to_textures = "./textures";
    --================================================================
    world = ode.dWorldCreate();
    ode.dWorldSetGravity(world, 0, 0, 0);
    ode.dWorldSetQuickStepNumIterations(world, 50); -- <-- increase for more stability
    
    space = ode.dSimpleSpaceCreate(nil);
    contactgroup = ode.dJointGroupCreate(0);
    local ground = ode.dCreatePlane(space, 0, 0, 1, 0);
    
    place_cards();
     
    --drawstuff
    ds.dsSimulationLoop (0,nil,640, 480, func[0]);
    --================================================================
    levels = 0;
    place_cards();
  
    --================================================================
    ode.dJointGroupDestroy(contactgroup);
    ode.dWorldDestroy(world);
    ode.dGeomDestroy(ground);
    ode.dSpaceDestroy(space);
    ode.dCloseODE();
end
--====================================================================
main ()
--====================================================================

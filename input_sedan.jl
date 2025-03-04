# suspension geometry is measured using an origin on the ground at the tire contact point, with x pointing forward, y pointing to the left, z up
# define the left side only, right is mirrored, so y values should all be -ve, z all +ve
# front spring geometry, lower end is fixed to the lower A-arm, and the upper end is fixed to the chassis; as the spring becomes more vertical, it becomes effectively stiffer at the wheel; similarly, as it moves outward closer to the wheel, it also becomes effectively stiffer at the wheel

@kwdef mutable struct susp # suspension properties
    r = 0.3
    ubj = [0, -0.15, 2 * r] # upper ball joint
    lbj = [0, -0.1, r / 2] # lower ball joint
    uaapf = [0.15, -0.4, 2 * r] # upper a-arm pivot, front
    uaapr = [-0.15, -0.4, 2 * r] # upper a-arm pivot, rear
    laapf = [0.15, -0.5, r / 2] # lower a-arm pivot, front
    laapr = [0, -0.5, r / 2] # lower a-arm pivot, rear
    itre = [-0.1, -0.45, r / 2] # inner tie rod end
    otre = [-0.1, -0.12, r / 2] # outer tie rod end
    sle = [0, -0.2, r / 2] # spring lower end
    sue = [0, -0.3, 2 * r]  # spring upper end
end

@kwdef mutable struct params_list
    m = 1800.0 # mass
    u = 0.0 # speed
    a = 1.5 # wheelbase, front
    b = 1.5
    kf = 30000.0 # suspension stiffness, front
    kr = 30000.0
    cf = 2500.0 # suspension damping, front
    cr = 2500.0
    krf = 500.0 # anti-roll stiffness, front
    krr = 500.0
    muf = 20.0 # unsprung mass, front
    mur = 20.0
    hG = 0.5 # mass centre height
    cfy = 40000.0 # cornering stiffness, front
    cry = 40000.0
    Ix = 818.0 # moments of inertia
    Iy = 3267.0
    Iz = 3508.0
    kt = 180000.0 # tire vertical stiffness
    r = 0.3 # wheel radius
    Iw = 1.0 # wheel rotary inertia
    tf = 1.5
    tr = 1.5
    front = susp(; r)
    rear = susp(; r)
end


function input_sedan(; kwargs...)

    the_system = mbd_system("EoM Sedan")
    the_system.scratch = params_list(; kwargs...)

    (; m, u, a, b, kf, kr, cf, cr, krf, krr, muf, mur, hG, cfy, cry, Ix, Iy, Iz, kt, r, Iw, tf, tr, front, rear) = the_system.scratch

    item = body("Chassis")
    item.mass = m
    item.moments_of_inertia = [Ix, Iy, Iz]
    item.location = [0, 0, hG]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = actuator("Chassis X")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    add_item!(item, the_system)

    item = actuator("Chassis Y")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    add_item!(item, the_system)

    item = actuator("Chassis Z")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    add_item!(item, the_system)

    item = actuator("Chassis L")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    item.twist = 1
    add_item!(item, the_system)

    item = actuator("Chassis M")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.twist = 1
    add_item!(item, the_system)

    item = actuator("Chassis N")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    add_item!(item, the_system)

    item = sensor("Chassis u")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    item.order = 2
    add_item!(item, the_system)

    item = sensor("Chassis v")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.order = 2
    add_item!(item, the_system)

    item = sensor("Chassis w")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.order = 2
    add_item!(item, the_system)

    item = sensor("Chassis p")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    item.twist = 1
    item.order = 2
    add_item!(item, the_system)

    item = sensor("Chassis q")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.twist = 1
    item.order = 2
    add_item!(item, the_system)

    item = sensor("Chassis r")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    item.order = 2
    add_item!(item, the_system)

    item = sensor("Chassis z")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    add_item!(item, the_system)

    item = sensor("Chassis ϕ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    item.twist = 1
    add_item!(item, the_system)

    item = sensor("Chassis θ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.twist = 1
    add_item!(item, the_system)

    item = sensor("Chassis ψ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    add_item!(item, the_system)

    item = sensor("Passenger z accln")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [a / 2, 0, hG]
    item.location[2] = [a / 2, 0, hG - 0.1]
    item.order = 3
    add_item!(item, the_system)

## Drivetrain

    item = body("Front differential")
    item.location = [a, 0, r]
    add_item!(item, the_system)

    item = body("Front diff gear")
    item.location = [a + 0.1, 0, r]
    add_item!(item, the_system)

    item = rigid_point("Front diff bearing")
    item.body[1] = "Front differential"
    item.body[2] = "Chassis"
    item.location = [a, 0, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("Front diff gear bearing")
    item.body[1] = "Front diff gear"
    item.body[2] = "Front differential"
    item.location = [a + 0.1, 0, r]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator("Front axle torque")
    item.body[1] = "Front differential"
    item.body[2] = "Chassis"
    item.location[1] = [a, 0, r]
    item.location[2] = [a, -0.1, r]
    item.twist = 1
    item.units = "N*m"
    add_item!(item, the_system)

    item = sensor("Front axle speed")
    item.body[1] = "Front differential"
    item.body[2] = "Chassis"
    item.location[1] = [a, 0, r]
    item.location[2] = [a, -0.1, r]
    item.twist = 1
    item.order = 2
    item.units = "rad/s"
    add_item!(item, the_system)

    item = body("Rear differential")
    item.location = [-b, 0, r]
    add_item!(item, the_system)

    item = body("Rear diff gear")
    item.location = [-b + 0.1, 0, r]
    add_item!(item, the_system)

    item = rigid_point("Rear diff bearing")
    item.body[1] = "Rear differential"
    item.body[2] = "Chassis"
    item.location = [-b, 0, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("Rear diff gear bearing")
    item.body[1] = "Rear diff gear"
    item.body[2] = "Rear differential"
    item.location = [-b + 0.1, 0, r]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator("Rear axle torque")
    item.body[1] = "Rear differential"
    item.body[2] = "Chassis"
    item.location[1] = [-b, 0, r]
    item.location[2] = [-b, -0.1, r]
    item.twist = 1
    item.units = "N*m"
    add_item!(item, the_system)

    item = sensor("Rear axle speed")
    item.body[1] = "Rear differential"
    item.body[2] = "Chassis"
    item.location[1] = [-b, 0, r]
    item.location[2] = [-b, -0.1, r]
    item.twist = 1
    item.order = 2
    item.units = "rad/s"
    add_item!(item, the_system)

    ####

    item = body("LF Wheel+hub")
    item.moments_of_inertia = [0, Iw, 0]
    item.location = [a, tf / 2, r]
    item.velocity = [u, 0, 0]
    item.angular_velocity = [0, u / r, 0]
    add_item!(item, the_system)

    item = body("LF Upright")
    item.mass = muf
    item.location = [a, tf / 2 - 0.1, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = rigid_point("LF Wheel bearing")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "LF Upright"
    item.location = [a, tf / 2, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LF Tire")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location = [a, tf / 2, 0]
    item.stiffness = [kt, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    if u > 0
        item = flex_point("LF Tire")
        item.body[1] = "LF Wheel+hub"
        item.body[2] = "ground"
        item.location = [a, tf / 2, 0]
        item.damping = [cfy / u, 0]
        item.forces = 1
        item.moments = 0
        item.axis = [0, 1, 0]
        add_item!(item, the_system)

        item = flex_point("LF Tire")
        item.body[1] = "LF Wheel+hub"
        item.body[2] = "ground"
        item.location = [a, tf / 2, 0]
        item.damping = [cfy / u, 0]
        item.forces = 1
        item.moments = 0
        item.axis = [1, 0, 0]
        add_item!(item, the_system)
    end

    item = link("LF Tie-rod")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = front.itre + [a, tf / 2, 0]
    item.location[2] = front.otre + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Upper A-arm, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = front.uaapf + [a, tf / 2, 0]
    item.location[2] = front.ubj + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Upper A-arm, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = front.uaapr + [a, tf / 2, 0]
    item.location[2] = front.ubj + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = body("LF Lower A-arm")
    item.location = (front.lbj + front.laapr + front.laapf) / 3 + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Lower ball joint")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "LF Upright"
    item.location = front.lbj + [a, tf / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LF Lower A-arm pivot, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Lower A-arm"
    item.location = front.laapf + [a, tf / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LF Lower A-arm pivot, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Lower A-arm"
    item.location = front.laapr + [a, tf / 2, 0]
    item.forces = 2
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = body("LF Anti-roll arm")
    item.location = [a - 0.25, 0.5, front.sle[3] - 0.05]
    add_item!(item, the_system)

    item = rigid_point("LF Anti-roll arm pivot")
    item.body[1] = "Chassis"
    item.body[2] = "LF Anti-roll arm"
    item.location = [a - 0.25, 0.5, front.sle[3] - 0.05]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LF Drop link")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "LF Anti-roll arm"
    item.location[1] = front.sle + [a, tf / 2, 0]
    item.location[2] = front.sle + [0, 0, -0.05] + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = spring("LF Suspension spring")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "Chassis"
    item.location[1] = front.sle + [a, tf / 2, 0]
    item.location[2] = front.sue + [a, tf / 2, 0]
    item.stiffness = kf
    item.damping = cf
    add_item!(item, the_system)

    item = spring("Front Anti-roll bar")
    item.body[1] = "LF Anti-roll arm"
    item.body[2] = "RF Anti-roll arm"
    item.location[1] = [a - 0.25, 0.45, front.sle[3] - 0.05]
    item.location[2] = item.location[1] .* [1, -1, 1]
    item.stiffness = krf
    item.twist = 1
    add_item!(item, the_system)

    item = actuator("LF Tire X")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a - 0.1, tf / 2, 0]
    add_item!(item, the_system)

    item = actuator("LF Tire Y")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2 - 0.1, 0]
    item.units = "N"
    add_item!(item, the_system)

    item = actuator("LF Tire Z")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2, -0.1]
    item.units = "N"
    add_item!(item, the_system)

    item = actuator("LF Brake")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "LF Upright"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a, tf / 2 + 0.1, r]
    item.twist = 1
    item.units = "N*m"
    add_item!(item, the_system)

    item = actuator("LF Tire z_0")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2, -0.1]
    item.gain = kt
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("LF Tire u")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a - 0.1, tf / 2, 0]
    item.order = 2
    item.units = "m/s"
    add_item!(item, the_system)

    item = sensor("LF Tire v")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2 - 0.1, 0]
    item.order = 2
    item.units = "m/s"
    add_item!(item, the_system)

    item = sensor("LF Tire Z")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2, -0.1]
    item.gain = kt
    item.actuator = "LF Tire z_0"
    item.actuator_gain = -kt
    item.units = "N"
    add_item!(item, the_system)

    item = sensor("LF Wheel travel speed")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a - 0.1, tf / 2, r]
    item.order = 2
    item.units = "m/s"
    add_item!(item, the_system)

    item = sensor("LF Camber")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a - 0.1, tf / 2, r]
    item.twist = 1
    item.units = "rad"
    add_item!(item, the_system)

    item = sensor("LF Roll steer")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "Chassis"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a, tf / 2, r - 0.1]
    item.twist = 1
    item.units = "rad"
    add_item!(item, the_system)

    item = body("LF Axle")
    item.location = [a, tf / 4, r]
    add_item!(item, the_system)

    item = body("LF Stub axle")
    item.location = [a, 0.1, r]
    add_item!(item, the_system)

    item = rigid_point("LF CV joint")
    item.body[1] = "LF Axle"
    item.body[2] = "LF Wheel+hub"
    item.location = [a, tf / 2 - 0.1, r]
    item.forces = 3
    item.moments = 1
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Plunge joint")
    item.body[1] = "LF Axle"
    item.body[2] = "LF Stub axle"
    item.location = [a, 0.1, r]
    item.forces = 2
    item.moments = 1
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Axle bearing")
    item.body[1] = "LF Stub axle"
    item.body[2] = "Front differential"
    item.location = [a, 0.1, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Diff gear")
    item.body[1] = "LF Stub axle"
    item.body[2] = "Front diff gear"
    item.location = [a + 0.1, 0.1, r]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    #####

    item = body("LR Wheel+hub")
    item.moments_of_inertia = [0, Iw, 0]
    item.location = [-b, tr / 2, r]
    item.velocity = [u, 0, 0]
    item.angular_velocity = [0, u / r, 0]
    add_item!(item, the_system)

    item = body("LR Upright")
    item.mass = mur
    item.location = [-b, tr / 2 - 0.1, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = rigid_point("LR Wheel bearing")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "LR Upright"
    item.location = [-b, tr / 2, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LR Tire")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location = [-b, tr / 2, 0]
    item.stiffness = [kt, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    if u > 0
        item = flex_point("LR Tire")
        item.body[1] = "LR Wheel+hub"
        item.body[2] = "ground"
        item.location = [-b, tr / 2, 0]
        item.damping = [cry / u, 0]
        item.forces = 1
        item.moments = 0
        item.axis = [0, 1, 0]
        add_item!(item, the_system)

        item = flex_point("LR Tire")
        item.body[1] = "LR Wheel+hub"
        item.body[2] = "ground"
        item.location = [-b, tr / 2, 0]
        item.damping = [cry / u, 0]
        item.forces = 1
        item.moments = 0
        item.axis = [1, 0, 0]
        add_item!(item, the_system)
    end

    item = link("LR Tie-rod")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = rear.itre + [-b, tr / 2, 0]
    item.location[2] = rear.otre + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Upper A-arm, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = rear.uaapf + [-b, tr / 2, 0]
    item.location[2] = rear.ubj + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Upper A-arm, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = rear.uaapr + [-b, tr / 2, 0]
    item.location[2] = rear.ubj + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = body("LR Lower A-arm")
    item.location = (rear.lbj + rear.laapr + rear.laapf) / 3 + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = rigid_point("LR Lower ball joint")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "LR Upright"
    item.location = rear.lbj + [-b, tr / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LR Lower A-arm pivot, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Lower A-arm"
    item.location = rear.laapf + [-b, tr / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LR Lower A-arm pivot, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Lower A-arm"
    item.location = rear.laapr + [-b, tr / 2, 0]
    item.forces = 2
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = body("LR Anti-roll arm")
    item.location = [-b + 0.25, 0.5, rear.sle[3] - 0.05]
    add_item!(item, the_system)

    item = rigid_point("LR Anti-roll arm pivot")
    item.body[1] = "Chassis"
    item.body[2] = "LR Anti-roll arm"
    item.location = [-b + 0.25, 0.5, rear.sle[3] - 0.05]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LR Drop link")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "LR Anti-roll arm"
    item.location[1] = rear.sle + [-b, tr / 2, 0]
    item.location[2] = rear.sle + [0, 0, -0.05] + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = spring("LR Suspension spring")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "Chassis"
    item.location[1] = rear.sle + [-b, tr / 2, 0]
    item.location[2] = rear.sue + [-b, tr / 2, 0]
    item.stiffness = kr
    item.damping = cr
    add_item!(item, the_system)

    item = spring("Rear anti-roll bar")
    item.body[1] = "LR Anti-roll arm"
    item.body[2] = "RR Anti-roll arm"
    item.location[1] = [-b + 0.25, 0.45, rear.sle[3] - 0.05]
    item.location[2] = item.location[1] .* [1, -1, 1]
    item.stiffness = krr
    item.twist = 1
    add_item!(item, the_system)

    item = actuator("LR Tire X")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b - 0.1, tr / 2, 0]
    item.units = "N"
    add_item!(item, the_system)

    item = actuator("LR Tire Y")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2 - 0.1, 0]
    item.units = "N"
    add_item!(item, the_system)

    item = actuator("LR Tire Z")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2, -0.1]
    item.units = "N"
    add_item!(item, the_system)

    item = actuator("LR brake")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "LR Upright"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b, tr / 2 + 0.1, r]
    item.twist = 1
    item.units = "N*m"
    add_item!(item, the_system)

    item = actuator("LR Tire z_0")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2, -0.1]
    item.gain = kt
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("LR Tire u")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b - 0.1, tr / 2, 0]
    item.order = 2
    item.units = "m/s"
    add_item!(item, the_system)

    item = sensor("LR Tire v")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2 - 0.1, 0]
    item.order = 2
    item.units = "m/s"
    add_item!(item, the_system)

    item = sensor("LR Tire Z")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2, -0.1]
    item.gain = kt
    item.actuator = "LR Tire z_0"
    item.actuator_gain = -kt
    item.units = "N"
    add_item!(item, the_system)

    item = sensor("LR Wheel travel speed")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b - 0.1, tr / 2, r]
    item.order = 2
    item.units = "m/s"
    add_item!(item, the_system)

    item = sensor("LR Camber")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b - 0.1, tr / 2, r]
    item.twist = 1
    item.units = "rad"
    add_item!(item, the_system)

    item = sensor("LR Roll steer")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "Chassis"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b, tr / 2, r - 0.1]
    item.twist = 1
    item.units = "rad"
    add_item!(item, the_system)

    item = body("LR Axle")
    item.location = [-b, tr / 4, r]
    add_item!(item, the_system)

    item = body("LR Stub axle")
    item.location = [-b, 0.1, r]
    add_item!(item, the_system)

    item = rigid_point("LR CV joint")
    item.body[1] = "LR Axle"
    item.body[2] = "LR Wheel+hub"
    item.location = [-b, tr / 2 - 0.1, r]
    item.forces = 3
    item.moments = 1
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR Plunge joint")
    item.body[1] = "LR Axle"
    item.body[2] = "LR Stub axle"
    item.location = [-b, 0.1, r]
    item.forces = 2
    item.moments = 1
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR Axle bearing")
    item.body[1] = "LR Stub axle"
    item.body[2] = "Rear differential"
    item.location = [-b, 0.1, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR Diff gear")
    item.body[1] = "LR Stub axle"
    item.body[2] = "Rear diff gear"
    item.location = [-b + 0.1, 0.1, r]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    ####

    # reflect all LF or LR items in y axis
    mirror!(the_system)
    
    the_system

end


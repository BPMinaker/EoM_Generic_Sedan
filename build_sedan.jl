using EoM, EoM_X3D, DelimitedFiles

# read student chosen params
include("input_sedan.jl")

r = 0.314 # tire size - fixed
include("my_specs.jl")

# everything below here is fixed
wb = 2.63
b = fwf * 2.63
a = wb - b
tf = 1.52
tr = 1.52
m = 1571
muf = 50
mur = 50
hG = 0.439
cfy = 0 # unused as u = 0
cry = 0
Ix = 461
Iy = 1848
Iz = 2045
kt = 200000
Iw = 1.75

system = input_sedan(; a, b, m, muf, mur, hG, cfy, cry, Ix, Iy, Iz, kt, Iw, tf, tr, kf, kr, cf, cr, krf, krr, front, rear)
output = run_eom!(system)
eom_draw(system)
result = analyze(output)

# write results and animate mode shapes, if you want
#summarize(system, result, format=:html)
#animate_modes(system, result)

# write system matrices to files
(; A, B, C, D) = result.ss_eqns
writedlm(joinpath("data", "A.dat"), A)
writedlm(joinpath("data", "B.dat"), B)
writedlm(joinpath("data", "C.dat"), C)
writedlm(joinpath("data", "D.dat"), D)

# get list of flex_point names
flex_names = getproperty.(system.flex_points, :name)
# find which ones are tires
n = findall(contains.(flex_names, "Tire"))
# find the preload in this tires
Z0 = vcat(getproperty.(system.flex_points[n], :preload)...)
# save to file
writedlm(joinpath("data", "Z0.dat"), Z0)
# find the mass, and write to file
m = sum(Z0) / 9.81
writedlm(joinpath("data", "mass.dat"), m)
# find the chassis, get its inertia matrix, and write to file
body_names = getproperty.(system.bodys, :name)
n = findfirst(contains.(body_names, "Chassis"))
IG = EoM.inertia_mtx(system.bodys[n])
writedlm(joinpath("data", "IG.dat"), IG)
writedlm(joinpath("data", "l.dat"), a + b)
writedlm(joinpath("data", "brake.dat"), fbf)

# generate the road profile and write to file
L = 500
zofx = random_road(; class=3, L)
ll = 0:L/1000:L
zz = zofx.(ll)
writedlm(joinpath("data", "z.dat"), [ll zz])

println("Done.")

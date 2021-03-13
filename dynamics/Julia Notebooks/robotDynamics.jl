### A Pluto.jl notebook ###
# v0.12.21

using Markdown
using InteractiveUtils

# ╔═╡ b862d976-8059-11eb-2c7c-f1402af1bb11
begin
	using RigidBodyDynamics
	using Plots
	using MeshCatMechanisms
end

# ╔═╡ 57a69aee-805b-11eb-1e7a-abc13a0badfb
md"# Rigid Body Dynamics for Pendulum and Xarm-6"

# ╔═╡ 96a4e62c-805d-11eb-3939-5d656749a674
md" Load the URDF file"

# ╔═╡ db3ff676-805a-11eb-2966-5926c509020b
pendulumURDF = "models/doublependulum.urdf";

# ╔═╡ 87b0620a-805c-11eb-1e81-cdf21e24ad78
doublePendulum = parse_urdf(Float64, pendulumURDF);

# ╔═╡ 93db1e76-805c-11eb-3999-dbda40a65e49
state = MechanismState(doublePendulum);

# ╔═╡ aa68ee02-805c-11eb-0fd9-c71f7aeb675d
vis = MechanismVisualizer(doublePendulum, URDFVisuals(pendulumURDF));

# ╔═╡ a9c7a67c-805d-11eb-3f8a-8701ad011f88
md"This opens the visualizer in another tab"

# ╔═╡ bb0fac5a-805c-11eb-25f1-4137ef49905f
open(vis)

# ╔═╡ 443e046c-805e-11eb-0e20-85302254cd44
set_configuration!(state, [1.0, -1.5]); set_configuration!(vis, configuration(state));

# ╔═╡ f9b69c7a-805c-11eb-30af-2162c42cd00a
ts, qs, vs = simulate(state, 5., Δt = 1e-3);

# ╔═╡ 84b174c6-805d-11eb-26a2-37bbb2ddcee9
shoulder_angles = collect(q[1] for q in qs);

# ╔═╡ d2f89ab0-805d-11eb-3b72-c1d200e5a262
md"Plot the shoulder angles from the initial conditions."

# ╔═╡ d9ae1ccc-805d-11eb-367e-6342214c5953
plot(ts, shoulder_angles)

# ╔═╡ 2a44082e-8066-11eb-0946-bf70a70c10ec
bodies(doublePendulum)

# ╔═╡ 2daf00ac-8066-11eb-3c3a-4d26ba59f7b0
M = mass_matrix(state)

# ╔═╡ 52c46852-8066-11eb-1ed8-3b192a278f1e
md"Goal is to get
$M(q) \ddot{q} + C(q, \dot{q}) =  \tau$ equation form. "

# ╔═╡ 325a9310-8067-11eb-1b4d-f17b41939cb8
v̇ = similar(velocity(state));

# ╔═╡ cc20d338-8067-11eb-33f0-9917435df3d9
v̇ .= [2.;3.]; τ = inverse_dynamics(state, v̇); @show τ

# ╔═╡ 45e86aaa-8068-11eb-05cb-cf5bbb0ab72f
result = DynamicsResult{Float64}(doublePendulum);

# ╔═╡ 534d53cc-8068-11eb-3980-e561ad8383ff
dynamics!(result, state)

# ╔═╡ 5a88c4e6-8068-11eb-19c7-41b63711a5a0
@show result.v̇

# ╔═╡ 489661f2-8346-11eb-1815-d9c606c3e54b
print(doublePendulum.graph)

# ╔═╡ 9a24821c-8346-11eb-0bc8-43982924c7f2
@show doublePendulum

# ╔═╡ Cell order:
# ╟─57a69aee-805b-11eb-1e7a-abc13a0badfb
# ╠═b862d976-8059-11eb-2c7c-f1402af1bb11
# ╟─96a4e62c-805d-11eb-3939-5d656749a674
# ╠═db3ff676-805a-11eb-2966-5926c509020b
# ╠═87b0620a-805c-11eb-1e81-cdf21e24ad78
# ╠═93db1e76-805c-11eb-3999-dbda40a65e49
# ╠═aa68ee02-805c-11eb-0fd9-c71f7aeb675d
# ╟─a9c7a67c-805d-11eb-3f8a-8701ad011f88
# ╠═bb0fac5a-805c-11eb-25f1-4137ef49905f
# ╠═443e046c-805e-11eb-0e20-85302254cd44
# ╠═f9b69c7a-805c-11eb-30af-2162c42cd00a
# ╠═84b174c6-805d-11eb-26a2-37bbb2ddcee9
# ╟─d2f89ab0-805d-11eb-3b72-c1d200e5a262
# ╠═d9ae1ccc-805d-11eb-367e-6342214c5953
# ╠═2a44082e-8066-11eb-0946-bf70a70c10ec
# ╠═2daf00ac-8066-11eb-3c3a-4d26ba59f7b0
# ╟─52c46852-8066-11eb-1ed8-3b192a278f1e
# ╠═325a9310-8067-11eb-1b4d-f17b41939cb8
# ╠═cc20d338-8067-11eb-33f0-9917435df3d9
# ╠═45e86aaa-8068-11eb-05cb-cf5bbb0ab72f
# ╠═534d53cc-8068-11eb-3980-e561ad8383ff
# ╠═5a88c4e6-8068-11eb-19c7-41b63711a5a0
# ╠═489661f2-8346-11eb-1815-d9c606c3e54b
# ╠═9a24821c-8346-11eb-0bc8-43982924c7f2

### A Pluto.jl notebook ###
# v0.12.21

using Markdown
using InteractiveUtils

# ╔═╡ d5fa5d6e-8406-11eb-3554-a37701037f0d
using LinearAlgebra

# ╔═╡ 38d8ad54-8408-11eb-0586-2b3a0ec6feb8
md"# Learning SVA"

# ╔═╡ 42652a50-8408-11eb-0631-a90e6526b3e0
md"First define the motion vector and force vector objects, then define their multiplication and addition"

# ╔═╡ e0f0c3e8-8406-11eb-0668-29967ebc9029
struct MotionVector
	data
end


# ╔═╡ 308a4a52-8407-11eb-2982-2511989ec4e6
struct ForceVector
	data
end

# ╔═╡ b2027f08-8407-11eb-0d3c-bb45274e1898
Base.:*(a::Union{MotionVector, ForceVector}, b::Union{MotionVector, ForceVector}) = a.data' * b.data

# ╔═╡ 2bf1282a-840b-11eb-0004-b7fcf8cb686c
Base.:+(a::Union{MotionVector, ForceVector}, b::Union{MotionVector, ForceVector}) = a.data + b.data

# ╔═╡ a11c615a-840b-11eb-1286-277230b9d7fa
Base.:-(a::Union{MotionVector, ForceVector}, b::Union{MotionVector, ForceVector}) = a.data - b.data

# ╔═╡ 35871a06-8407-11eb-3f3b-ddaa1404e80f
v = MotionVector([0, 0, 0, 1, 2, 3])

# ╔═╡ ab7495ea-8407-11eb-0d7a-d15eed360ec2
f = ForceVector([1, 1, 1, 1, 1, 1])

# ╔═╡ 2b3c25e0-8408-11eb-16ed-0f1bd5477457
md"We can test out the communativity of this like so: "

# ╔═╡ ae78027a-8407-11eb-22e0-e9a6c0afd065
v*f == f*v

# ╔═╡ 34912354-840b-11eb-2498-bb777315d8b9
f+f

# ╔═╡ 6ee3162a-8408-11eb-02b9-e7e96eb47e06
md"Since motion and force vectors are in dual coordinate systems, they obey a different coordinate transformation rule using $X$."

# ╔═╡ 907009a4-8408-11eb-3595-0f7734ff591f
struct MotionTransform
	data
end

# ╔═╡ 0935a902-8409-11eb-35e8-8f7c044aa32c
# always create a force transform from a motion transform
struct ForceTransform
	data
	
	function ForceTransform(a::MotionTransform)
		new(inv(a.data)')
	end
end

# ╔═╡ 41664dfe-8409-11eb-02ec-2bb36f74ce9d
Base.:*(a::MotionTransform, b::MotionVector) = a.data * b.data

# ╔═╡ d33147f2-8409-11eb-1a8f-45184c4c8a35
Base.:*(a::ForceTransform, b::ForceVector) = a.data * b.data

# ╔═╡ 8d247424-8408-11eb-19b4-cf97dc988ae9
X₁ = MotionTransform(I(6))

# ╔═╡ 9eff7242-8409-11eb-05ac-5b24dc5a7e12
X¹ = ForceTransform(X₁)

# ╔═╡ b640c8ac-8409-11eb-1fe6-83dd3d0f63c9
X₁*v

# ╔═╡ 7faf22f2-8409-11eb-01c2-5be0daf45b93
X¹*f

# ╔═╡ 792f551e-8409-11eb-2336-f9e88e886c62
md"## Using Spatial Vectors"

# ╔═╡ 294be04a-8408-11eb-0d2e-011bf4299aa0
md"What do the rules of classical mechanics look like in spatial vector form?"

# ╔═╡ d64f89ba-840b-11eb-2d07-61007c134f8b
md"
Note: the following is lifted directly from [2]. 

* _Relative Velocity_: If bodies $B_1, B_2$ have velocities $v_1, v_2$, then relative velocity of $B_2$ with respect to $B_1$ is $v_{rel} = v_2 - v_1$. Conversely, $v_2 = v_1 + v_{ref}$.

* _Summation of forces_: If forces $f_1, f_2$ act on the same rigid body, then they are equivalent to a single force $f_{tot} = f_1 + f_2$. 

* _Action and reaction_: If body $B_1 exerts a force $f$ on body $B_2$, then $B_2$ exerts a force $-f$ on $B_1$. 

* _Scalar Product_: If a force $f$ acts on a body having a velocity of $v$, then the power delivered by that force is $f \cdot v$.

* _Scalar Multiplication_: This operation affects a spatial vector’s magnitudes but not its directed line. If a spatial vector $s$ is characterized by magnitudes $m_1$ and $m_2$ and line $l$, then $\alpha s$ is characterized by $\alpha m_1, \alpha m_2, l$. A body having a velocity of $\alpha v$ makes the same infinitesimal motion over a period of $\delta t$ as a body with a velocity of $v$ makes over a period of $\alpha \delta t$; and a force $\beta f$ delivers $\beta$ times as much power as a force $f$ acting on the same body. So, $(\alpha v) \cdot (\beta f ) = \alpha \beta (v \cdot f )$.

* _Differentiation_: Spatial vectors are differentiated just like any other vector. The derivative of a motino vector is a motion vector, and the derivative of a force vector is a force vector. If $m \in M^6, f \in F^6$ are fixed in a body having a velocity of $v$, then $\dot{m} = v \times m, \dot{f} = v \times^* f$. 

* _Acceleration_: Spatial acceleration is the time derivative of spatial velocity ($a = \dot v$). For example, if $v_2 = v_1 + v_{rel}$, then $a_2 = a_1 + a_{rel}$. Spatial accelerations are elements of $M^6$ and therefore obey the same coordinate transformation rule as velocities. 

* _Summation of Inertias_: If bodies $B_1, B_2$ having inertias $I_1, I_2$ are rigidly connected to form a single composite rigid body, then the inertia of the composite is $I_{tot} = I_1 + I_2$.

* _Momentum_: If a rigid body has a velocity of $v$ and an inertia of $I$, then its momentum is $Iv$. 

* _Equation of motion_: The total force acting on a rigid body equals its rate of change of momentum. $f = d(Iv)/dt = Ia + v \times^* Iv$.

* _Motion constraints_: If the relative velocity of two rigid bodies is constrained to lie in a subspace $S \subseteq M^6$, then the motion constraint is implemented by a constraint force lying in the subspace $T= \{ f \in F^6 | f \cdot v = 0 \quad \forall \quad  v \in S\}$. This is a statement of D'Alembert's principle of virtual work (or Jourdain's principle of virtual power) expressed using spatial vectors. 
"

# ╔═╡ e0e74178-8407-11eb-28b4-a1c8fb8a50ad
md"## Plucker transforms"

# ╔═╡ de6cb2ae-8411-11eb-04cf-b791eda6fb77
md" Say we had a frame $B$ that was located $r = [2, 4, 1]^T$ away from from $A$ in a cartesian frame. Then the rotation is: "

# ╔═╡ 0b146cfa-8412-11eb-03f8-55415e7a6318
E₂ = Matrix(I, 3, 3)

# ╔═╡ 1f2ad1f0-8412-11eb-22b8-5fcd1200df45
md" and the translation matrix is given by $[r]_{\times}$" 

# ╔═╡ 1a253bba-8413-11eb-08c5-95e2bcfe8b5a
r = [2, 4, 1]

# ╔═╡ 27793678-8412-11eb-2e44-516a701b1643
skew(r) = [   0 -r[3] r[2]
		    r[3]   0 -r[1]
		   -r[2] r[1]   0]; r_skew = skew(r)

# ╔═╡ 8cf8fe08-8411-11eb-3668-79e8613819e0
begin
X₂ = [       E₂ zeros(3, 3)
 	 zeros(3, 3)         E₂]

X₂ = MotionTransform(X₂*[Matrix(I, 3, 3)     zeros(3, 3)
	           					-r_skew Matrix(I, 3, 3)])
end

# ╔═╡ 9d816da0-840c-11eb-007d-a3958af6c953
md"## References"

# ╔═╡ a816e272-840c-11eb-2326-af0cc485d084
md"
1. R. Featherstone, Rigid Body Dynamics Algorithms. New York: SpringerVerlag, 2008.
2. R. Featherstone, A beginners guide to 6D Vectors Part1 & 2. IEEE Robotics and Automation Magazine, 2010.
"

# ╔═╡ Cell order:
# ╟─38d8ad54-8408-11eb-0586-2b3a0ec6feb8
# ╠═d5fa5d6e-8406-11eb-3554-a37701037f0d
# ╟─42652a50-8408-11eb-0631-a90e6526b3e0
# ╠═e0f0c3e8-8406-11eb-0668-29967ebc9029
# ╠═308a4a52-8407-11eb-2982-2511989ec4e6
# ╠═b2027f08-8407-11eb-0d3c-bb45274e1898
# ╠═2bf1282a-840b-11eb-0004-b7fcf8cb686c
# ╠═a11c615a-840b-11eb-1286-277230b9d7fa
# ╠═35871a06-8407-11eb-3f3b-ddaa1404e80f
# ╠═ab7495ea-8407-11eb-0d7a-d15eed360ec2
# ╟─2b3c25e0-8408-11eb-16ed-0f1bd5477457
# ╠═ae78027a-8407-11eb-22e0-e9a6c0afd065
# ╠═34912354-840b-11eb-2498-bb777315d8b9
# ╟─6ee3162a-8408-11eb-02b9-e7e96eb47e06
# ╠═907009a4-8408-11eb-3595-0f7734ff591f
# ╠═0935a902-8409-11eb-35e8-8f7c044aa32c
# ╠═41664dfe-8409-11eb-02ec-2bb36f74ce9d
# ╠═d33147f2-8409-11eb-1a8f-45184c4c8a35
# ╠═8d247424-8408-11eb-19b4-cf97dc988ae9
# ╠═9eff7242-8409-11eb-05ac-5b24dc5a7e12
# ╠═b640c8ac-8409-11eb-1fe6-83dd3d0f63c9
# ╠═7faf22f2-8409-11eb-01c2-5be0daf45b93
# ╟─792f551e-8409-11eb-2336-f9e88e886c62
# ╟─294be04a-8408-11eb-0d2e-011bf4299aa0
# ╟─d64f89ba-840b-11eb-2d07-61007c134f8b
# ╟─e0e74178-8407-11eb-28b4-a1c8fb8a50ad
# ╟─de6cb2ae-8411-11eb-04cf-b791eda6fb77
# ╠═0b146cfa-8412-11eb-03f8-55415e7a6318
# ╟─1f2ad1f0-8412-11eb-22b8-5fcd1200df45
# ╠═1a253bba-8413-11eb-08c5-95e2bcfe8b5a
# ╠═27793678-8412-11eb-2e44-516a701b1643
# ╠═8cf8fe08-8411-11eb-3668-79e8613819e0
# ╟─9d816da0-840c-11eb-007d-a3958af6c953
# ╟─a816e272-840c-11eb-2326-af0cc485d084

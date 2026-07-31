# Motor inertia

Notation: $q_i$ is the joint angle, $\theta_r$ the rotor angle, $N$ the gear ratio, $J_r$ the motor-side rotor inertia, and $h_i^{i-1}$ the joint spin axis in body frame $\{i\}$. Numbers use the XM430-W350 ($N = 353.5$) and joint 4 of the OpenMANIPULATOR-X.



## 1. Deriving $J_{\mathrm{arm}}$

A rigid gearbox constrains the rotor to the joint:

$$\theta_r = N q_i, \qquad \dot\theta_r = N \dot q_i, \qquad \ddot\theta_r = N \ddot q_i .$$

Writing the rotor kinetic energy in the joint coordinate,

$$T_r = \tfrac{1}{2} J_r \dot\theta_r^{\,2} = \tfrac{1}{2} J_r \left( N \dot q_i \right)^2 = \tfrac{1}{2} \underbrace{\left( N^2 J_r \right)}_{J_{\mathrm{arm}}} \dot q_i^{\,2} .$$

Since $T = \tfrac12 \dot q^\top M(q) \dot q$, whatever multiplies $\tfrac12 \dot q_i^{\,2}$ is the corresponding diagonal entry of $M$. Hence $J_{\mathrm{arm}} = N^2 J_r$: one factor of $N$ from rate amplification, the second because energy is quadratic in rate.

Equivalently by torque balance, with $\tau_m = J_r \ddot\theta_r + \tau_{\mathrm{load}}/N$ and $\tau_i = N \tau_m$:

$$\tau_i = N \left( J_r N \ddot q_i + \frac{\tau_{\mathrm{load}}}{N} \right) = N^2 J_r \, \ddot q_i + \tau_{\mathrm{load}} .$$

With $J_r \approx 1.6 \times 10^{-7}\ \mathrm{kg\,m^2}$, this gives $J_{\mathrm{arm}} \approx 2.0 \times 10^{-2}\ \mathrm{kg\,m^2}$ — three orders of magnitude above the joint-4 link axial inertia $I_{\mathrm{ax}} \approx 2.0 \times 10^{-5}\ \mathrm{kg\,m^2}$.

It enters the model as a rank-one term along the spin axis, at the body-inertia level so that $C(q,\dot q)$ is derived from the same $M$:

$$I_i^{\,i} = \operatorname{diag}\!\left( I_\perp,\, I_\perp,\, I_{\mathrm{ax}} \right) + J_{\mathrm{arm}} \, h_i^{i-1} \big( h_i^{i-1} \big)^{\!\top} .$$



## 2. Effect on angular acceleration

Commanded torque must accelerate rotor and link together — there is no path to the output shaft that bypasses the rotor:

$$\ddot q_4 = \frac{\tau_4}{I_{\mathrm{ax}} + N^2 J_r} , \qquad M_{44} \approx 2.0 \times 10^{-5} + 2.0 \times 10^{-2} \approx 2.0 \times 10^{-2}\ \mathrm{kg\,m^2} .$$

| Model | $M_{44}\ [\mathrm{kg\,m^2}]$ | $1/M_{44}\ [\mathrm{rad\,s^{-2}/N\,m}]$ | $\ddot q$ at stall torque $4.1\ \mathrm{N\,m}$ |
|---|---|---|---|
| Without $J_{\mathrm{arm}}$ | $2.0 \times 10^{-5}$ | $5.0 \times 10^{4}$ | $2 \times 10^{5}\ \mathrm{rad/s^2}$ |
| With $J_{\mathrm{arm}}$ | $2.0 \times 10^{-2}$ | $5.0 \times 10^{1}$ | $2 \times 10^{2}\ \mathrm{rad/s^2}$ |

The no-$J_{\mathrm{arm}}$ column is unphysical: no XM430 accelerates at $10^5\ \mathrm{rad/s^2}$. That model treats the joint as near-massless, so gains that are reasonable on hardware are over-aggressive by $\sim 10^3$ in simulation.

Because $J_{\mathrm{arm}} \gg I_{\mathrm{ax}}$ at every joint, $M_\rho \approx N^2 J_r \, \mathbf{I}_n$ and $\kappa(M_\rho) \to 1$, so one gain set works across all joints and $\lVert \tilde{M}^{-1} \rVert$ no longer amplifies model error along the end-effector heading direction.

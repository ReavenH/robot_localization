$$
% KF robot
\left[
\begin{matrix}
x' \\
y' \\
\theta' \\
\end{matrix}
\right]
=
\left[
\begin{matrix}
x \\
y \\
\theta \\
\end{matrix}
\right]
+ 
\left[
\begin{matrix}
\hat{\delta}_{trans} \cos(\theta + \hat{\delta}_{rot1}) \\
\hat{\delta}_{trans} \sin(\theta + \hat{\delta}_{rot1}) \\
\hat{\delta}_{rot1} + \hat{\delta}_{rot2}
\end{matrix}
\right]
$$

![image-20240327121642897](F:\Hiwonder_SharedFiles\robot_localization\KF_Robot.assets\image-20240327121642897.png)
$$
S = 
\left[
\begin{matrix}
x \\
y \\
z \\
\phi \\
\theta \\
\psi
\end{matrix}
\right]
;
U= 
\left[
\begin{matrix}
\delta_{\psi1} \\
\delta_{trans} \\
\delta_{\psi2} \\
\end{matrix}
\right]
;
A \in \mathbb{R}^{6\times 6}
;
B \in \mathbb{R}^{6\times 3}
\\
S'=
A\cdot S
+
B\cdot
U
$$



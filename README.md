# Modelagem e Controle nãolinear do Quadrirrotor em Matlab-Simulink
Matlab 9.4.0.813654 (R2018a)

![Imagem_simulink](figures/modelo_drone.png)

O modelo Simulink `UAV_naolinear.slx` e o arquivo `modelo_naolinear_params.m` representam a modelagem de um veículo aéreo não tripulado tipo VANT quadrirrotor.

O modelo implementado é o seguinte:
```math
\begin{align}
    \begin{split}
		\dot{{\xi}}&=v\\
		\dot{v}&= -[0~0~{g}]^T+(1/m){{R}}_B^I[0~0~{T}_b]^T\\
		\dot{\omega}&=I^{-1}\left({M}-{\omega}\times{{I}}{\omega}\right)\\
		\dot{{\eta}}&={\mathcal{W}}{{\omega}}
    \end{split}    
\end{align}   
```
onde ${\xi}=(x,y,z)$ representa a posição do veículo no referencial inercial e ${v}=(v_x,v_y,v_z)$ representa a velocidade translacional. O vetor ${\omega}=(p,q,r)$ é a velocidade angular e ${\eta}=(\phi, \theta, \psi)$ são os ângulos de Euler.


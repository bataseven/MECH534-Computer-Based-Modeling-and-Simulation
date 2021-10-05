close all

syms x1(t) x2(t)

b1 = 1;
b2 = 1;
c1 = 1;
c2 = 1;

ode1 = diff(x1) == (b1 - c1 * x2) * x1;
ode2 = diff(x2) == (-b2 + c2 * x1) * x2;

odes = [ode1; ode2];

[VF,Sbs] = odeToVectorField(odes);
odsefcn = matlabFunction(VF, 'Vars',{'T','Y'});

sol = ode45(odsefcn,[0 10],[0.5 0.5]) % Change the solver to ode23 or ode113 to see different results

figure
hold on
plot(sol.x, sol.y(1,:),'r')
plot(sol.x, sol.y(2,:),'b')
title(strcat(sol.solver," Solution"))
xlabel('Time (s)')
ylabel('Population')
legend('Prey','Predator','location','southeast')


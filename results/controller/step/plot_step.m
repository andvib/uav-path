close all;
clear all;

data = load('step_response.mat');
data2 = load('step_aileron.mat');

start_idx = 4000;
end_idx = 15000;

p_N = data.states.Position.p_N.Data(start_idx:end_idx);
p_E = data.states.Position.p_E.Data(start_idx:end_idx);

phi = data.states.Attitude.phi.Data(start_idx:end_idx);
theta = data.states.Attitude.theta.Data(start_idx:end_idx);
psi = data.states.Attitude.psi.Data(start_idx:end_idx);

e = data.control.Data(start_idx:end_idx,1);
a = data.control.Data(start_idx:end_idx,2);
r = data.control.Data(start_idx:end_idx,3);
t = data.control.Data(start_idx:end_idx,4);

velocities = data.states.Velocity;

course_angle = data.airdata.Course_angle.Data(start_idx:end_idx);
course_desired = data.course_desired.Data(start_idx:end_idx);
course_aileron = data2.airdata.Course_angle.Data(start_idx:end_idx);

time = data.states.Position.p_N.Time(start_idx:end_idx)-start_idx*0.01;

figure(1);
grid on;
hold on;
plot(time,phi);
plot(time, theta);
plot(time,psi);
legend('\phi', '\theta', '\psi');
xlim([0 110]);
xlabel('Time [s]');
ylabel('Angle [rad]');

figure(2);
grid on;
hold on;
plot(time, course_angle);
plot(time, course_desired, 'k');
plot(time, course_aileron, '--k');
legend('Course Angle \chi','Course Angle Aileron','Desired course', 'Location', 'SE');
ylabel('Angle [rad]');
xlabel('Time [s]');
xlim([0 110]);

figure(3);
grid on;
hold on;
plot(time, e);
plot(time, a);
plot(time, r);
plot(time, t);
xlabel('Time [s]');
xlim([0 110]);
legend('Elevator', 'Aileron', 'Rudder', 'Throttle');

figure(5);
grid on;
hold on;
plot(data.states.Velocity.u);
plot(data.states.Velocity.v);
plot(data.states.Velocity.w);
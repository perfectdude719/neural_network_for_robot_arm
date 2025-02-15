a = arduino();
clear a;
a = arduino('COM5', 'Uno', 'Libraries', 'Servo');
s = servo(a, 'D9')
clear s;
s = servo(a, 'D9', 'MinPulseDuration', 700e-6, 'MaxPulseDuration', 2300e-6)
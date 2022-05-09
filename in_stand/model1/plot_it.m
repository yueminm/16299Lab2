function p2( data )
  %
figure(1)
hold off
plot(data(:,11),'k')
hold on
plot(data(:,14),'b')
ascale = 5.575380291904716e-05; % convert Y accelerometer signals to radians
plot(ascale*(data(:,9)-data(1,9)),'r')
title( 'In stand with wheels fixed: encoder (in radians) and body angle (also radians)' )
ylabel( 'radians' )
xlabel( 'sample' )
legend( 'wheel encoders', 'accelerometers', 'online Kf','Location','SouthWest' )
% axis([1 length(data) -0.6 0.6]) % for ff000
axis([1 length(data) -3 3]) % for ff002

figure(2)
hold off
plot(data(:,12))             
hold on
plot(data(:,15),'Linewidth',3)
title( 'In stand with wheels fixed: velocity from encoders and gyro' )
ylabel( 'radians/second' )
legend( 'velocity from encoders','velocity from gyro','Location','NorthWest' )
axis([1 length(data) -10 10])


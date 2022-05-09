function new_data = regress( data, filter_cutoff )

% Use this filter
if filter_cutoff < 1.0 
[b_filter a_filter] = butter( 1, filter_cutoff );
else % no filter at all
  b_filter = [1 0];
  a_filter = [1 0];
end

Ts = 0.003;

% number of points in data
N = length(data);

% get position in radians
wheel = pi*(data(:,2)+data(:,3))/1560;

wf = filtfilt( b_filter, a_filter, wheel );

% estimate wheel velocity
v(N) = 0; % allocate velocity array
for ( i = 3:N )
  v(i-1) = (wf(i) - wf(i-2))/(2*Ts);
end
v(1) = v(2); % take care of ends
v(N) = v(N-1);

% estimate wheel acceleration
a(N) = 0; % allocate velocity array
for ( i = 3:N )
  a(i-1) = (wf(i) - 2*wf(i-1) + wf(i-2))/(Ts*Ts);
end
a(1) = a(2); % take care of ends
a(N) = a(N-1);

% pull out command array
commands = data(:,4);

commandsf = filtfilt( b_filter, a_filter, commands );

body_angle(N) = 0;
for i = 1:N
  body_angle(i) = atan2( data(i,6), data(i,7) );
end

body_anglef = filtfilt( b_filter, a_filter, body_angle );

gscale = 1.352011295799561e-04; % convert gyro to radians/second
body_velocityf = filtfilt( b_filter, a_filter, gscale*data(:,8) );

% estimate body_acceleration
ba(N) = 0; % allocate velocity array
for ( i = 3:N )
  ba(i-1) = (body_velocityf(i) - body_velocityf(i-2))/(2*Ts);
end
ba(1) = ba(2); % take care of ends
ba(N) = ba(N-1);

new_data = data;

[nr nc] = size(data);

new_data(:,nc+1) = wheel;
new_data(:,nc+2) = wf;
new_data(:,nc+3) = v;
new_data(:,nc+4) = a;
new_data(:,nc+5) = body_anglef;
new_data(:,nc+6) = body_velocityf;
new_data(:,nc+7) = ba;

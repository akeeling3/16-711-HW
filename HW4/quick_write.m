% Code used for quickly writing all deliverables

% Part 1
for n = 1:10
   fn_in = strcat('part1/data/p1n0',num2str(n-1));
   fn_out = strcat('part1/p1a0',num2str(n-1));
   states = kf_p1(fn_in);
   save(fn_out, 'states', '-ascii')
end

% Part 2
for n = 1:10
   fn_in = strcat('part2/data/p2n0',num2str(n-1));
   fn_out = strcat('part2/p2a0',num2str(n-1));
   states = kf_p2(fn_in);
   save(fn_out, 'states', '-ascii')
end

% Part 3
for n = 1:10
   fn_in = strcat('part3/data/p3n0',num2str(n-1));
   fn_out = strcat('part3/p3a0',num2str(n-1));
   states = kf_p3(fn_in);
   save(fn_out, 'states', '-ascii')
end
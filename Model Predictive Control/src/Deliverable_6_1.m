%%% DELIVERABLE 6.1 %%%
set(0,'defaultfigurecolor',[1 1 1]);
quad = Quad();

CTRL = ctrl_NMPC(quad);

sim = quad.sim(CTRL);
quad.plot(sim)
function [ world ] = get_world_estimate(wRk, bRc, lambda, xf,yf, bPc, wPk)


world = wRk*(bRc*exp(lambda)*[xf;yf;1]+bPc)+wPk;
%world = world/(world(3))

end


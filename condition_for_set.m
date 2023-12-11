function cond = condition_for_set(pv, x, y)
l = length(pv);
conditions = strings(l,1);
% fake_zero = 10e-5;
for i=1:length(pv) - 1
    conditions_n = simplify((pv(2,i) - pv(2,i+1))*x + (pv(1,i+1) - pv(1,i))*y <= - (pv(1,i)*pv(2,i+1) - pv(1,i+1)*pv(2,i)));
    conditions(i) = string(conditions_n);
end
conditions_n = simplify((pv(2,l) - pv(2,1))*x + (pv(1,1) - pv(1,l))*y <= - (pv(1,l)*pv(2,1) - pv(1,1)*pv(2,l)));
conditions(l) = string(conditions_n);
cond = join(conditions, ' & ');


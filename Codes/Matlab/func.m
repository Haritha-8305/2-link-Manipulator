function der_S = func(t,state,q1_des,q2_des)
    % System parameters
    m1=5;
    m2=3;
    l1=0.25;
    l2=0.15;
    g=9.81;

    % Controller gains
    kp1 = 800;
    kd1 = 0;
    ki1 = 0;
    kp2 = 800;
    kd2 = 0;
    ki2 = 0;

    % Extracting state variables
    q1=state(1);
    q2=state(2);
    q1dot=state(3);
    q2dot=state(4);
    neg_int_q1=state(5);
    neg_int_q2=state(6);

    % Equations of motion -
    % M(q) — Inertia Matrix (Mass Matrix)
    M11 = ((m1+m2)*(l1^2))+(m2*l2*(l2+2*l1*cos(q2)));
    M12 = m2*l2*(l2+l1*cos(q2));
    M21 = m2*l2*(l2+l1*cos(q2));
    M22 = m2*(l2^2);
    M = [M11,M12; M21,M22];
    % C(q,q˙) — Coriolis and Centrifugal Matrix
    c11 = -m2*l1*l2*sin(q2)*q2dot;
    c12 = -m2*l1*l2*sin(q2)*(q1dot+q2dot);
    c21 = 0;
    c22 = m2*l1*l2*sin(q2)*q1dot;
    C = [c11,c12; c21,c22];
    % G(q) — Gravity Torque Vector    
    g1 = m1*l1*g*cos(q1) + m2*g*(l2*cos(q1 + q2)+l1*cos(q1));
    g2 = m2*g*l2*cos(q1 + q2);
    G = [g1; g2];

    %Torque calculations
    q1_error = q1-q1_des;
    q2_error = q2-q2_des;
    tau1 = -kp1*q1_error - kd1*q1dot + ki1*neg_int_q1;
    tau2 = -kp2*q2_error - kd2*q2dot + ki2*neg_int_q2;
    Tau = [tau1; tau2];

    % Solve for q1_ddot and q2_ddot
    q_ddot = M \ (Tau-C*[q1dot; q2dot]-G);

    % State derivatives
    der_S = [q1dot;q2dot;q_ddot(1);q_ddot(2);-q1_error;-q2_error];
end
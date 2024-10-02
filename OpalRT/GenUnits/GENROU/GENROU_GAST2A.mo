within OpalRT.GenUnits.GENROU;
class GENROU_GAST2A
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 900 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 200 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.3 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.04 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.4 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 2.6 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.67 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.62 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.3 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.3 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.01 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.04 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.1 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.2 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  //
  // GAST2A Parameters
  parameter String TG_ID = M_ID "Machine Identifier" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MBASE = SB "base Power, MVA" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real W_tg = 10 "governor gain (1/droop) (on turbine rating)" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real X_tg = 0.01 "governor lead time constant, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Y_tg = 0.5 "(> 0) governor lag time constant, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Z_tg = 0.1 "governor mode: 1 Droop, 0 ISO" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real ETD_tg = 0.1 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TCD_tg = 0.01 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TRATE_tg = 100 "trubine rating, MW" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T_tg = 0.3 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MAX_tg = 6 "Maximum limit on turbine rating, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real MIN_tg = -6 "Minimum limit on turbine rating, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real ECR_tg = 0.1 "sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K3_tg = 0.1 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real a_tg = 0.1 "(> 0) valve positioner" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real b_tg = 0.5 "(> 0) valve positioner, sec." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real c_tg = 0.1 "valve positioner" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Tf_tg = 0.04 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Kf_tg = 0.04 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K5_tg = 0.02 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K4_tg = 2 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T3_tg = 5 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T4_tg = 0.25 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real Tt_tg = 150 "(> 0)" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real T5_tg = 0.3 "(> 0), sec" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real af1_tg = 100 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real bf1_tg = 150 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real af2_tg = -0.2 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real bf2_tg = 0.3 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real cf2_tg = 0.1 annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TR_T_tg = 250 "Rated temperature" annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real K6_tg = 0.03 "Minimum fuel flow, p.u." annotation(Dialog(tab = "GAST2A Parameters"));
  parameter Real TC_T_tg = 150 "Temperature control" annotation(Dialog(tab = "GAST2A Parameters"));
  //
  OpalRT.Electrical.Control.TurbineGovernor.GAST2A gast2a1(ID = TG_ID, W = W_tg, X = X_tg, Y = Y_tg, Z = Z_tg, ETD = ETD_tg, TCD = TCD_tg, TRATE = TRATE_tg, T = T_tg, MAX = MAX_tg, MIN = MIN_tg, ECR = ECR_tg, K3 = K3_tg, a = a_tg, b = b_tg, c = c_tg, Tf = Tf_tg, Kf = Kf_tg, K5 = K5_tg, K4 = K4_tg, T3 = T3_tg, T4 = T4_tg, Tt = Tt_tg, T5 = T5_tg, af1 = af1_tg, bf1 = bf1_tg, af2 = af2_tg, bf2 = bf2_tg, cf2 = cf2_tg, TR = TR_T_tg, K6 = K6_tg, TC = TC_T_tg) annotation(Placement(visible = true, transformation(origin={-84,14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin={-46,16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin={-144,34}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin={-144,34}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-20, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-200, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genrou1.VI, gast2a1.VI) annotation(Line(points = {{-36, 16}, {-26.7252, 16},
          {-26.7252, -8.32742}, {-102.64, -8.32742}, {-102.64, 14.1373}, {-94.3129,
          14.1373}, {-94.3129, 14.1373}}, color = {0, 0, 127}));
  connect(genrou1.MBASE, gast2a1.MBASE) annotation(Line(points = {{-36, 10.6},
          {-29.2428, 10.6}, {-29.2428, -5.22885}, {-100.123, -5.22885}, {-100.123,
          9.87671}, {-94.3129, 9.87671}, {-94.3129, 9.87671}}, color = {0, 0, 127}));
  connect(gast2a1.dGREF, dGREF) annotation(Line(points = {{-94, 22}, {-144.699, 22},
          {-144.699, 30.5935}, {-144.699, 30.5935}}, color = {0, 0, 127}));
  connect(gast2a1.PMECH, genrou1.PMECH) annotation (Line(points = {{-74,22},{
          -68,22},{-68,10},{-56,10}},
                                 color = {0,0,127}));
  connect(genrou1.PMECH0, gast2a1.PMECH0) annotation (Line(points = {{-56,8},{
          -66,8},{-66,0},{-74,0},{-74,20}},color = {0,0,127}));
  connect(genrou1.SLIP, gast2a1.SLIP) annotation (Line(points = {{-36,8},{-32,8},
          {-32,-2},{-98,-2},{-98,6},{-94,6}}, color = {0,0,127}));
  connect(TRIP, genrou1.TRIP)
    annotation (Line(points = {{-20,60},{-46,60},{-46,26}}, color={0,0,0}));
  connect(genrou1.EFD0, genrou1.EFD) annotation (Line(points = {{-56.2,16},{-60,
          16},{-60,18.8},{-56,18.8}}, color = {0,0,127}));
  connect(bus0, genrou1.p)
    annotation (Line(points = {{0,0},{-46,0},{-46,6}}, color={0,0,0}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-60.16, 5.76}, extent = {{-116.79, 25.76}, {128.56, -28.59}}, textString = "GENROU_GAST2A"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN")}));
end GENROU_GAST2A;

within OpalRT.GenUnits.GENCLS;
class GENCLS_TGOV1
  parameter Real partType = 1;
  //
  parameter Integer IBUS = 100 "Located system bus";
  parameter String M_ID = "M1" "Machine Identifier";
  parameter Real P_gen = 900;
  parameter Real Q_gen = 200;
  parameter Real Vt_abs = 1.03;
  parameter Real Vt_ang = -10.96;
  parameter Real SB = 1000;
  parameter Real fn = 50;
  parameter Real H = 5;
  parameter Real D = 2;
  parameter Real ZSOURCE_RE = 0;
  parameter Real ZSOURCE_IM = 0;
  // GAST Parameters
  parameter Real R_tg = 0.01 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T1_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMAX_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real VMIN_tg = -1 annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real T2_tg = 0.01;
  parameter Real T3_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "TGOV1 Parameters"));
  parameter Real Dt_tg = 1 annotation(Dialog(tab = "TGOV1 Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENCLS gencls1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, H = H, D = D, ZSOURCE_RE = ZSOURCE_RE, ZSOURCE_IM = ZSOURCE_IM) annotation(Placement(visible = true, transformation(origin = {25, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 40}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.TGOV1 tgov11(R = R_tg, T1 = T1_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, T2 = T2_tg, T3 = T3_tg, Dt = Dt_tg) annotation(Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
equation
  connect(tgov11.PMECH, gencls1.PMECH) annotation(Line(points = {{-5, 12}, {1.00489, 12}, {1.00489, -23.6149}, {9.71392, -23.6149}, {9.71392, -23.6149}}, color = {0, 0, 127}));
  connect(gencls1.PMECH0, tgov11.PMECH0) annotation(Line(points = {{10, -27}, {-1.67481, -27}, {-1.67481, 9.04399}, {-3.85207, 9.04399}, {-3.85207, 9.04399}}, color = {0, 0, 127}));
  connect(gencls1.MBASE, tgov11.MBASE) annotation(Line(points = {{40, -23.1}, {53.2231, -23.1}, {53.2231, -41.6529}, {-49.2562, -41.6529}, {-49.2562, -5.95041}, {-34.3802, -5.95041}, {-34.3802, -5.95041}}, color = {0, 0, 127}));
  connect(gencls1.SLIP, tgov11.SLIP) annotation(Line(points = {{40, -27}, {49.5868, -27}, {49.5868, -40.9917}, {-46.6116, -40.9917}, {-46.6116, -11.9008}, {-35.3719, -11.9008}, {-35.3719, -11.9008}}, color = {0, 0, 127}));
  connect(gencls1.VI, tgov11.VI) annotation(Line(points = {{40, -15}, {58.5124, -15}, {58.5124, -43.3058}, {-52.2314, -43.3058}, {-52.2314, -0.330579}, {-36.6942, -0.330579}, {-36.6942, -0.330579}}, color = {0, 0, 127}));
  connect(tgov11.dGREF, dGREF) annotation(Line(points = {{-35, 12}, {-80, 12}, {-80, 33.0579}, {-80, 33.0579}}, color = {0, 0, 127}));
  connect(TRIP, gencls1.TRIP) annotation(Line(points = {{20, 40}, {25.5125, 40}, {25.5125, 0}, {25, 0}}));
  connect(gencls1.p, bus0) annotation(Line(points = {{25, -30}, {54.8975, -30}, {54.8975, -39.18}, {100, -39.18}, {100, -40}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Text(origin = {-70.9533, 80.4127}, extent = {{-13.33, 7.29}, {15.8357, -9.34011}}, textString = "dGREF"), Text(origin = {-70.32, -58.36}, extent = {{-13.33, 7.29}, {15.84, -9.34}}, textString = "TRIP"), Text(origin = {-31.32, 0.57}, extent = {{-59.79, 33.83}, {121.75, -33.15}}, textString = "GENCLS_TGOV1"), Rectangle(origin = {0.113895, -0.455581}, extent = {{-96.9248, 93.8497}, {96.9248, -93.8497}})}));
end GENCLS_TGOV1;

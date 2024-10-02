within OpalRT.GenUnits.GENCLS;
class GENCLS_GAST
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
  parameter Real R_tg = 0.01 "Speed droop" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T1_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T2_tg = 0.01 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real T3_tg = 0.3 "(>0) (sec)" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real AT_tg = 0.12 "Ambient temperature load limit" annotation(Dialog(tab = "GAST Parameters"));
  parameter Real KT_tg = 0.2 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMAX_tg = 0.12 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real VMIN_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  parameter Real DTURB_tg = 0.01 annotation(Dialog(tab = "GAST Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENCLS gencls1(IBUS = IBUS, ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, H = H, D = D, ZSOURCE_RE = ZSOURCE_RE, ZSOURCE_IM = ZSOURCE_IM) annotation(Placement(visible = true, transformation(origin = {25, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.GAST gast1(R = R_tg, T1 = T1_tg, T2 = T2_tg, T3 = T3_tg, AT = AT_tg, KT = KT_tg, VMAX = VMAX_tg, VMIN = VMIN_tg, DTURB = DTURB_tg, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input
  OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin = {-80, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input
  OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {20, 40}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-12.5, -12.5}, {12.5, 12.5}}, rotation = 0)));
equation
  connect(gencls1.VI, gast1.VI) annotation(Line(points = {{40, -15}, {46.2725,
         -15}, {46.2725, -44.2159}, {-64.2674, -44.2159}, {-64.2674, -0.257069},
         {-56.2982, -0.257069}, {-56.2982, -0.257069}}, color = {0, 0, 127}));
  connect(gencls1.MBASE, gast1.MBASE) annotation(Line(points = {{40, -23.1},
         {42.9306, -23.1}, {42.9306, -39.8458}, {-62.7249, -39.8458}, {-62.7249,
         -5.65553}, {-54.4987, -5.65553}, {-54.4987, -5.65553}}, color = {0, 0, 127}));
  connect(gencls1.PMECH0, gast1.PMECH0) annotation (Line(points = {{10,-27},{
          -65.3759,-27},{-65.3759,-11.6173},{-25,-11.6173},{-25,9}},
        color = {0,0,127}));
  connect(gast1.dGREF, dGREF) annotation(Line(points = {{-55, 12}, {-80.7198, 12},
         {-80.7198, 34.4473}, {-80.7198, 34.4473}}, color = {0, 0, 127}));
  connect(TRIP, gencls1.TRIP) annotation(Line(points = {{20,40},{25.5125,40},{
          25.5125,0},{25,0}}));
  connect(gencls1.p, bus0) annotation(Line(points = {{25,-30},{54.8975,-30},{
          54.8975,-39.18},{100,-39.18},{100,-40}}));
  connect(gencls1.PMECH, gast1.PMECH) annotation (Line(points = {{10,-24},{-10,
          -24},{-10,12},{-25,12}}, color = {0,0,127}));
  connect(gencls1.SLIP, gast1.SLIP) annotation (Line(points = {{40,-27},{48,-27},
          {48,24},{-60,24},{-60,-12},{-55,-12}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-70.9533, 80.4127}, extent = {{-13.33, 7.29}, {15.8357, -9.34011}}, textString = "dGREF"), Text(origin = {-70.32, -58.36}, extent = {{-13.33, 7.29}, {15.84, -9.34}}, textString = "TRIP"), Text(origin = {-31.3238, 0.571162}, extent = {{-59.79, 33.83}, {121.749, -33.1466}}, textString = "GENCLS_GAST"), Rectangle(origin = {0.113895, -0.455581}, extent = {{-96.9248, 93.8497}, {96.9248, -93.8497}})}));
end GENCLS_GAST;

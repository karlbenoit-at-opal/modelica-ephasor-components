within OpalRT.GenUnits.GENCLS;
class GENCLS
  parameter Real partType = 1;
  parameter Integer IBUS = 100 "Located system bus";
  parameter String M_ID = "M1" "Machine Identifier";
  parameter Real P_gen = 900;
  parameter Real Vt_abs = 0.5;
  parameter Real Q_gen = 200;
  parameter Real Vt_ang = -1;
  parameter Real SB = 1000;
  parameter Real fn = 50;
  parameter Real H = 10;
  parameter Real D = 0;
  parameter Real ZSOURCE_RE = 0.1;
  parameter Real ZSOURCE_IM = 0.1;
  /*, Xd_p = Xd_p*/
  OpalRT.Electrical.Machine.SynchronousMachine.GENCLS gencls1(fn = fn, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, H = H, D = D, ZSOURCE_IM = ZSOURCE_IM, ZSOURCE_RE = ZSOURCE_RE, IBUS = IBUS, ID = M_ID) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  /*Xd_p = ZSOURCE_IM,*/
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-40, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {80, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(gencls1.PMECH0, gencls1.PMECH) annotation (Line(points = {{-15,-12},{
          -16.122,-12},{-16.122,-9},{-15,-9}}, color = {0,0,127}));
  connect(TRIP, gencls1.TRIP) annotation(Line(points = {{-40,40},{0.217865,40},
          {0.217865,15},{0,15}}));
  connect(gencls1.p, bus0) annotation(Line(points = {{0,-15},{79.5207,-15},{
          79.5207,-20},{80,-20}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-42.5932, 17.863}, extent = {{-25.6, 11.55}, {107.517, -44.0119}}, textString = "GENCLS"), Text(origin = {69.0632, -61.7728}, extent = {{-11.7679, 11.8688}, {11.55, -6.64}}, textString = "PIN"), Rectangle(origin = {-0.217865, 0.980392}, extent = {{-94.7712, 84.6405}, {94.7712, -84.6405}}), Text(origin = {-75.68, 61.6}, extent = {{-11.77, 11.87}, {29.09, -11.2}}, textString = "TRIP")}));
end GENCLS;

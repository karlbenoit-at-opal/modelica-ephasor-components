within OpalRT.GenUnits;
model CRANIT
  parameter Real partType = 1;
  parameter Real R = 1 "Resistance p.u.";
  parameter Real X_init = 1 "Reactance p.u.";
  parameter Real B = 1 "Shunt half susceptance p.u.";
  constant Real pi = Modelica.Constants.pi;
  parameter Real T1 = 1 "(s)";
  parameter Real T2 = 3 "(s)";
  parameter Real T3 = 3 "(s) (> 0)";
  parameter Real TW = 2 "(s) (> 0)";
  parameter Real K = 0.1;
  parameter Real Xmax = 0.1 "(pu) max. limit on output";
  parameter Real Xmin = -0.1 "(pu) min. limit on output";
  parameter Real INmax = 10 "(pu) max. limit on input signal";
  parameter Real INmin = -10 "(pu) min. limit on input signal";
  parameter Real Vmag0_from = 1;
  parameter Real Vang0_from = 10;
  parameter Real Vmag0_to = 0.95;
  parameter Real Vang0_to = 0;
  OpalRT.Electrical.Branch.HVDC.CRANIT   cranit1(R = R, X_init = X_init, B = B, T1 = T1, T2 = T2, T3 = T3, TW = TW, K = K, Xmax = Xmax, Xmin = Xmin, INmax = INmax, INmin = INmin, Vmag0_from = Vmag0_from, Vang0_from = Vang0_from, Vmag0_to = Vmag0_to, Vang0_to = Vang0_to) annotation(Placement(visible = true, transformation(origin = {5, 15}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {-60, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus1 annotation(Placement(visible = true, transformation(origin = {40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(bus1, cranit1.n) annotation(Line(points={{40,-20},{38.1597,-20},{
          38.1597,15.1556},{30,15.1556},{30,15}}));
  connect(bus0, cranit1.p) annotation(Line(points = {{-60, -20}, {-43.0311, -20}, {-43.0311, 15.1556}, {-20, 15.1556}, {-20, 15}}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {0.270636, 1.89445}, extent = {{-99.594, 27.8755}, {99.594, -27.8755}}), Text(origin = {-3.38572, 0.9459}, extent = {{-40.46, 13.13}, {40.46, -13.13}}, textString = "CRANIT"), Text(origin = {-46.7118, -5.04042}, extent = {{-40.46, 13.13}, {-16.1029, -0.95138}}, textString = "bus0"), Text(origin = {104.55, -5.61}, extent = {{-40.46, 13.13}, {-16.1, -0.95}}, textString = "bus1")}));
end CRANIT;

within OpalRT.NonElectrical.Math.Continuous.TransferFunctionWindup;
block PI_WindupLimit "PI controller with windup limiter"
  parameter Real KI "Integral gain", KP "Proportional gain";
  parameter Real MAX "Maximum limit of the output", MIN "minimum limit of the output";
  parameter Modelica.Blocks.Types.Init initType = Modelica.Blocks.Types.Init.InitialOutput "Type of initialization (1: no init, 2: steady state, 3,4: initial output)";
  parameter Real y_start = 0 "Initial or guess value of output (= state)";
  Modelica.Blocks.Continuous.Integrator integrator1(k = KI, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = if KI <> 0 then y_start else 0) annotation(Placement(visible = true, transformation(origin = {-40, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = KP) annotation(Placement(visible = true, transformation(origin = {-40, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Add add1 annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax = MAX, uMin = MIN) annotation(Placement(visible = true, transformation(origin = {40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y annotation(Placement(visible = true, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(integrator1.u, u) annotation(Line(points = {{-52, 40}, {-59.1017, 40}, {-59.1017, 0.945626}, {-80, 0.945626}, {-80, 0}}, color = {0, 0, 127}));
  connect(gain1.u, u) annotation(Line(points = {{-52, -20}, {-59.1017, -20}, {-59.1017, 0.945626}, {-80, 0.945626}, {-80, 0}}, color = {0, 0, 127}));
  connect(add1.u2, gain1.y) annotation(Line(points = {{-12, -6}, {-17.4941, -6}, {-17.4941, -20.5674}, {-29, -20.5674}, {-29, -20}}, color = {0, 0, 127}));
  connect(add1.u1, integrator1.y) annotation(Line(points = {{-12, 6}, {-16.7849, 6}, {-16.7849, 40.1891}, {-29, 40.1891}, {-29, 40}}, color = {0, 0, 127}));
  connect(limiter1.y, y) annotation(Line(points = {{51, 0}, {73.2861, 0}, {73.2861, 0}, {80, 0}}, color = {0, 0, 127}));
  connect(limiter1.u, add1.y) annotation(Line(points = {{28, 0}, {10.1655, 0}, {10.1655, 0}, {11, 0}}, color = {0, 0, 127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 50, Tolerance = 1e-06, Interval = 0.005), Documentation(info = "<html>
<p>The block diagram of this transfer function is shown below:</p>

<img src=\"modelica://OpalRT/resource/Math/Continuous/TransferFunctionWindup/PI_Windup_Limit.png\"
alt=\"PI_Windup_Limit.png\"><br>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.118203, 0.118203}, extent = {{-100.355, 57.8014}, {99.6454, -50.2364}}), Text(origin = {-28.37, -0.115461}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {9.46, -10.75}}, textString = "K"), Text(origin = {39.62, 4.76}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {9.46, -10.75}}, textString = "K"), Text(origin = {3.83, -25.36}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {-18.67, 4.38}}, textString = "P"), Text(origin = {68.75, -9.15}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {-18.67, 4.38}}, textString = "I"), Text(origin = {14.7989, -7.77719}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {9.46, -10.75}}, textString = "+"), Line(origin = {32.03, -3.31}, points = {{-18.0851, 0}, {8.86524, 0}}, color = {85, 0, 255}, thickness = 4), Text(origin = {51.44, -24.5528}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {-2.83, -4.13}}, textString = "s"), Line(origin = {53.93, 63.63}, points = {{25.4992, 6.11384}, {-15.3992, 6.11384}, {-25.0918, -5.70649}}, color = {85, 0, 255}, thickness = 3), Line(origin = {-49.71, -55.85}, rotation = 180, points = {{25.4992, 6.11384}, {-15.3992, 6.11384}, {-25.0918, -5.70649}}, color = {85, 0, 255}, thickness = 3), Text(origin = {82.45, 72.13}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {-1.89, -3.66}}, textString = "MAX"), Text(origin = {-30.22, -86.36}, lineColor = {85, 0, 255}, extent = {{-45.63, 26.12}, {-1.89, -3.66}}, textString = "MIN")}));
end PI_WindupLimit;

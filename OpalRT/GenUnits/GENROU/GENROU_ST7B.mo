within OpalRT.GenUnits.GENROU;
class GENROU_ST7B
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  //ST7B
  parameter Real TR_ex = 0.01 " regulator input filter time constant (s)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real TG_ex = 1 " lead time constant of voltage input (s)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real TF_ex = 1 " lag time constant of voltage input (s)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real VMAX_ex = 1.1 " voltage reference maximum limit (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real VMIN_ex = 0.9 " voltage reference minimum limit (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real KPA_ex = 40 "(>0) voltage regulator gain (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real VRMAX_ex = 5 " voltage regulator maximum limit (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real VRMIN_ex = -4.5 " voltage regulator minimum limit (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real KH_ex = 1 " feedback gain (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real KL_ex = 1 " feedback gain (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real TC_ex = 1 " lead time constant of voltage regulator (s)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real TB_ex = 1 " lag time constant of voltage regulator (s)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real KIA_ex = 1 "(>0) gain of the first order feedback block (p.u.)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real TIA_ex = 3 "(>0) time constant of the first order feedback block (s)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real OEL_ex = 1 "Flag (1,2 or 3)" annotation(Dialog(tab = "ST7B Parameters"));
  parameter Real UEL_ex = 1 "Flag (1,2 or 3)" annotation(Dialog(tab = "ST7B Parameters"));
  //
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {15, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {60, -40}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={10,9}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {0, 80}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  OpalRT.Electrical.Control.Excitation.ST7B st7b1(TR = TR_ex, TG = TG_ex, TF = TF_ex, VMAX = VMAX_ex, VMIN = VMIN_ex, KPA = KPA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KH = KH_ex, KL = KL_ex, TC = TC_ex, TB = TB_ex, KIA = KIA_ex, TIA = TIA_ex, OEL = OEL_ex, UEL = UEL_ex) annotation(Placement(visible = true, transformation(origin={-26,-2}, extent={{-16,-16},
            {16,16}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-88,-8}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-90, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = noVUEL) annotation(Placement(visible = true, transformation(origin={-69,34}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant2(k = noVOEL) annotation(Placement(visible = true, transformation(origin={-70,19}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-70,2}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  parameter Real noVUEL(fixed = false, start = 1);
  parameter Real noVOEL(fixed = false, start = 1);
initial equation
  noVUEL = if UEL_ex == 1 then 0 else -Modelica.Constants.inf;
  noVOEL = if OEL_ex == 1 then 0 else Modelica.Constants.inf;
equation
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{0,-27},{
          -2.71399,-27},{-2.71399,-24},{0,-24}}, color = {
          0,0,127}));
  connect(genrou1.p, bus0) annotation(Line(points = {{15,-30},{14.6701,-30},{
          14.6701,-41.6963},{60,-41.6963},{60,-40}}));
  connect(TRIP, genrou1.TRIP) annotation(Line(points = {{10,9},{14.9166,9},{
          14.9166,0},{15,0}}));
  connect(genrou1.EFD0, st7b1.EFD0) annotation (Line(points = {{-0.3,-15},{-5,
          -15},{-5,-14.8},{-10,-14.8}}, color = {0,0,127}));
  connect(st7b1.EFD, genrou1.EFD) annotation (Line(points = {{-10,-11.6},{-5,
          -11.6},{-5,-10.8},{0,-10.8}}, color = {0,0,127}));
  connect(st7b1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-10,-8.4},{
          -5.5,-8.4},{-5.5,-7.5},{-0.3,-7.5}}, color = {0,0,127}));
  connect(st7b1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-10,-4.24},
          {-4.5,-4.24},{-4.5,-3},{-0.3,-3}}, color = {0,0,127}));
  connect(genrou1.VI, st7b1.VI) annotation(Line(points = {{30, -15}, {42.6735, -15},
          {42.6735, 7.71208}, {-8.99743, 7.71208}, {-8.99743, 7.71208}}, color = {0, 0, 127}));
  connect(st7b1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-42,6},{-48,
          6},{-48,21},{37,21},{37,-7.5},{30,-7.5}}, color = {0,0,127}));
  connect(st7b1.dVREF, dVREF) annotation(Line(points = {{-42, -11.6}, {-87.1465, -11.6},
          {-87.1465, -9.76864}, {-87.1465, -9.76864}}, color = {0, 0, 127}));
  connect(st7b1.VUEL, constant1.y) annotation (Line(points = {{-42,1.2},{-52,
          1.2},{-52,34},{-63.5,34}}, color = {0,0,127}));
  connect(st7b1.VOEL, constant2.y) annotation (Line(points = {{-42,-3.28},{-56,
          -3.28},{-56,19},{-64.5,19}}, color = {0,0,127}));
  connect(st7b1.VOTHSG, const.y) annotation (Line(points = {{-42,-7.76},{-50,
          -7.76},{-50,-7},{-60,-7},{-60,2},{-64.5,2}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={ Rectangle(origin = {1.13895, -0.455581}, extent = {{-95.8998, 87.016}, {94.0775, -92.2551}}), Text(origin = {57.7941, -57.2869}, extent = {{25.85, -23.58}, {-20.1552, 17.8852}}, textString = "PIN"), Text(origin = {-5.13, 9.23}, extent = {{-73.46, 18.79}, {83.94, -29.04}}, textString = "GENROU_ST7B"), Text(origin = {-59.4304, -59.9804}, extent = {{59.0612, -18.9402}, {-20.16, 17.89}}, textString = "dVREF"), Text(origin = {-29.6516, 61.375}, extent = {{59.06, -18.94}, {3.03902, 10.3198}}, textString = "TRIP")}));
end GENROU_ST7B;

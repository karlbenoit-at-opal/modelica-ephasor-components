within OpalRT.GenUnits.GENROU;
class GENROU_SCRX
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1000 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 100 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 0.95 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -2 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1200 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 60 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 10.2 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.5 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 1.02 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.01 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 8.2 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 3 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.5 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.5231 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.361 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.41 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.2 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.5 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.6 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // SCRX Parameters
  parameter Real TA_TB_ex = 4 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 100 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 0.5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = -1.2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 2 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 3 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  //
  //
  //****************************
  //
  //
  OpalRT.Electrical.Control.Excitation.SCRX scrx1(
    IBUS = IBUS,
    ID = M_ID,
    TA_TB = TA_TB_ex,
    TB = TB_ex,
    K = K_ex,
    TE = TE_ex,
    EMIN = EMIN_ex,
    EMAX = EMAX_ex,
    CSWITCH = CSWITCH_ex,
    rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin={11,-1}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(
    IBUS = IBUS,
    ID = M_ID,
    P_gen = P_gen,
    Q_gen = Q_gen,
    Vt_abs = Vt_abs,
    Vt_ang = Vt_ang,
    SB = SB,
    fn = fn,
    ZSOURCE_RE = ZSOURCE_RE,
    Tdo_p = Tdo_p,
    Tdo_s = Tdo_s,
    Tqo_p = Tqo_p,
    Tqo_s = Tqo_s,
    H = H,
    D = D,
    Xd = Xd,
    Xq = Xq,
    Xd_p = Xd_p,
    Xq_p = Xq_p,
    Xd_s = Xd_s,
    Xl = Xl,
    S1 = S1,
    S12 = S12) annotation(Placement(visible = true, transformation(origin = {55, -15}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-22,2}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {60, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-76,-6}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(TRIP,genrou1. TRIP) annotation(Line(points = {{60,40},{59.4533,40},{
          59.4533,21.8679},{54.8975,21.8679},{54.8975,0},{55,0}}));
  connect(scrx1.dVREF, dVREF) annotation(Line(points = {{-4, -10}, {-75.0643, -10},
          {-75.0643, -9.51157}, {-75.0643, -9.51157}}, color = {0, 0, 127}));
  connect(scrx1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{26,-3.1},{
          34,-3.1},{34,-3},{39.7,-3}}, color = {0,0,127}));
  connect(scrx1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{26,-7},{34,
          -7},{34,-7.5},{39.7,-7.5}}, color = {0,0,127}));
  connect(scrx1.EFD, genrou1.EFD) annotation (Line(points = {{26,-10},{34,-10},
          {34,-10.8},{40,-10.8}}, color = {0,0,127}));
  connect(scrx1.EFD0, genrou1.EFD0) annotation (Line(points = {{26,-13},{34,-13},
          {34,-15},{39.7,-15}}, color = {0,0,127}));
  connect(genrou1.VI, scrx1.VI) annotation(Line(points = {{70, -15}, {84.0617, -15},
          {84.0617, 7.71208}, {26.7352, 7.71208}, {26.7352, 7.71208}}, color = {0, 0, 127}));
  connect(genrou1.XADIFD, scrx1.XADIFD) annotation (Line(points = {{70,-7.5},{
          74,-7.5},{74,-8},{78,-8},{78,20},{-10,20},{-10,6.5},{-4,6.5}},
        color = {0,0,127}));
  connect(const.y, scrx1.VUEL)
    annotation (Line(points = {{-16.5,2},{-4,2}}, color={0,0,127}));
  connect(scrx1.VOEL, scrx1.VUEL) annotation (Line(points = {{-4,-2.2},{-8,-2.2},
          {-8,-2},{-10,-2},{-10,2},{-4,2}}, color = {0,0,127}));
  connect(scrx1.VOTHSG, scrx1.VUEL) annotation (Line(points = {{-4,-6.4},{-10,
          -6.4},{-10,2},{-4,2}}, color = {0,0,127}));
  connect(bus0, genrou1.p)
    annotation (Line(points = {{100,-60},{55,-60},{55,-30}}, color={0,0,0}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{40,-27},{
          36,-27},{36,-24},{40,-24}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-52.4736, 7.94359}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_SCRX"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN")}));
end GENROU_SCRX;

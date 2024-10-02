within OpalRT.GenUnits.GENROU;
class GENROU_SCRX_IEEEST
  parameter Real partType = 1;
  constant Real pi = Modelica.Constants.pi;
  // GENROU Parameters
  /////////
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 572.93 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 796.24 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 6.56 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 1.5 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 3.03 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 2.95 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 2.82 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.697 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.697 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.2 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.35 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  //
  // SCRX Parameters
  ///////DATA BUS 31
  parameter Real TA_TB_ex = 1 "TA/TB" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TB_ex = 1 "(>0) (sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real K_ex = 1 annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real TE_ex = 5 "(sec)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMIN_ex = 0 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real EMAX_ex = 1 "(pu on EFD base)" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real CSWITCH_ex = 1 "0 for bus fed, 1 for solid fed" annotation(Dialog(tab = "SCRX Parameters"));
  parameter Real rc_rfd_ex = 1 "rc/rfd, 0 with negative field current capability (EX=EFD)" annotation(Dialog(tab = "SCRX Parameters"));
  //
  // IEEEST Parameters
  parameter Real A1_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A2_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A3_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A4_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A5_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real A6_pss = 0 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T1_pss = 0.03 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T2_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T3_pss = 0.02 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T4_pss = 0.01 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T5_pss = 0.2 "(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real T6_pss = 0.1 "(>0)(sec)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real KS_pss = -5 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMAX_pss = 6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real LSMIN_pss = -6 annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCU_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  parameter Real VCL_pss = 1 "(pu) (if equal zero, ignored)" annotation(Dialog(tab = "IEEEST Parameters"));
  // IEEEST ICONs
  parameter Real M0_pss = 1 "Stabilizer input code" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  parameter Real M1_pss = 1 "IB, remote bus number" annotation(Dialog(tab = "IEEEST Parameters", group = "ICONs"));
  //
  //
  //****************************
  //
  OpalRT.Electrical.Control.Stabilizer.IEEEST ieeest1(
    A1 = A1_pss,
    A2 = A2_pss,
    A3 = A3_pss,
    A4 = A4_pss,
    A5 = A5_pss,
    A6 = A6_pss,
    T1 = T1_pss,
    T2 = T2_pss,
    T3 = T3_pss,
    T4 = T4_pss,
    T5 = T5_pss,
    T6 = T6_pss,
    KS = KS_pss,
    LSMAX = LSMAX_pss,
    LSMIN = LSMIN_pss,
    VCU = VCU_pss,
    VCL = VCL_pss,
    M0 = M0_pss,
    M1 = M1_pss) annotation(Placement(visible = true, transformation(origin={-62,20}, extent = {{-15, -15}, {10, 10}}, rotation = 0)));
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
    S12 = S12) annotation(Placement(visible = true, transformation(origin={41,4}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-42,20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={86,-36}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin={86,-36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={41,64}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
    rc_rfd = rc_rfd_ex) annotation(Placement(visible = true, transformation(origin={0,16}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
    input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin = {-60, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ieeest1.VI2, ieeest1.VI) annotation(Line(points = {{-77, 12.5},
          {-81.6601, 12.5}, {-81.6601, 19.7964}, {-77.3622, 19.7964}, {-77.3622,
          19.7964}}, color = {0, 0, 127}));
  connect(genrou1.VI, ieeest1.VI) annotation(Line(points = {{56, 4}, {71.2409,
          4}, {71.2409, 41.2859}, {-89.3442, 41.2859}, {-89.3442, 19.7964},
          {-77.1017, 19.7964}, {-77.1017, 19.7964}}, color = {0, 0, 127}));
  connect(TRIP,genrou1. TRIP) annotation(Line(points = {{41,64},{41.1102,64},{
          41.1102,19},{41,19}}));
  connect(genrou1.p,bus0) annotation(Line(points = {{41,-11},{42,-11},{42,-36},
          {86,-36}}));
  connect(scrx1.EFD0, genrou1.EFD0)
    annotation (Line(points = {{15,4},{25.7,4}}, color={0,0,127}));
  connect(scrx1.EFD, genrou1.EFD) annotation (Line(points = {{15,7},{20.5,7},{
          20.5,8.2},{26,8.2}}, color = {0,0,127}));
  connect(scrx1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{15,10},{20,
          10},{20,11.5},{25.7,11.5}}, color = {0,0,127}));
  connect(scrx1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{15,13.9},{
          21.5,13.9},{21.5,16},{25.7,16}}, color = {0,0,127}));
  connect(genrou1.VI, scrx1.VI) annotation(Line(points = {{56, 4}, {71.2082, 4},
          {71.2082, 25.1928}, {15.1671, 25.1928}, {15.1671, 25.1928}}, color = {0, 0, 127}));
  connect(scrx1.XADIFD, genrou1.XADIFD) annotation (Line(points = {{-15,23.5},{
          -24,23.5},{-24,38},{64,38},{64,11.5},{56,11.5}}, color = {0,0,127}));
  connect(const.y, scrx1.VOEL) annotation (Line(points = {{-36.5,20},{-32,20},{
          -32,14.8},{-15,14.8}}, color = {0,0,127}));
  connect(scrx1.VUEL, scrx1.VOEL) annotation (Line(points = {{-15,19},{-32,19},
          {-32,14.8},{-15,14.8}}, color = {0,0,127}));
  connect(scrx1.dVREF, dVREF) annotation(Line(points = {{-15, 7}, {-35.4756, 7},
          {-35.4756, -39.3316}, {-56.5553, -39.3316}, {-56.5553, -39.3316}}, color = {0, 0, 127}));
  connect(ieeest1.VOTHSG, scrx1.VOTHSG) annotation (Line(points = {{-52,10},{
          -34,10},{-34,10.6},{-15,10.6}}, color = {0,0,127}));
  connect(genrou1.SLIP, ieeest1.PSS_AUX[1]) annotation(Line(points = {{56, -8}, {59.3891, -8},
          {59.3891, -7.9446}, {63.9475, -7.9446}, {63.9475, -14.8473}, {-79.967, -14.8473},
          {-79.967, 17.4521}, {-77.1017, 17.4521}, {-77.1017, 17.4521}}, color = {0, 0, 127}));
  connect(genrou1.AccPower, ieeest1.PSS_AUX[2]) annotation(Line(points = {{56, 0.1}, {63.9475,
          0.1}, {63.9475, -14.8473}, {-79.967, -14.8473}, {-79.967, 17.4521}, {-77.2319,
          17.4521}, {-77.2319, 17.4521}}, color = {0, 0, 127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{26,-8},{22,
          -8},{22,-5},{26,-5}}, color = {0,0,127}));
  connect(ieeest1.PSS_AUX2, ieeest1.PSS_AUX) annotation (Line(points = {{-76.75,
          10},{-80,10},{-80,17.5},{-77,17.5}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-51.82, 7.29}, extent = {{-38.61, 12.3}, {138.15, -25.51}}, textString = "GENROU_SCRX_IEEEST"), Rectangle(origin = {-0.683371, -0.569476}, extent = {{-99.5444, 98.7472}, {99.5444, -98.7472}}), Text(origin = {77.2165, -60.4778}, extent = {{-15.7169, 9.23023}, {4.55513, -5.58558}}, textString = "PIN")}));
end GENROU_SCRX_IEEEST;

(define (problem os-sequencedstrips-p190_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) 
(stacks-avail n0)

(waiting o1)
(includes o1 p63)(includes o1 p104)(includes o1 p119)(includes o1 p124)(includes o1 p136)(includes o1 p161)(includes o1 p167)(includes o1 p184)(includes o1 p185)(includes o1 p186)

(waiting o2)
(includes o2 p39)(includes o2 p51)(includes o2 p58)(includes o2 p115)(includes o2 p126)(includes o2 p147)

(waiting o3)
(includes o3 p62)(includes o3 p118)(includes o3 p150)(includes o3 p190)

(waiting o4)
(includes o4 p37)(includes o4 p44)(includes o4 p90)(includes o4 p92)(includes o4 p104)(includes o4 p107)(includes o4 p132)

(waiting o5)
(includes o5 p33)(includes o5 p34)(includes o5 p147)(includes o5 p156)(includes o5 p171)

(waiting o6)
(includes o6 p76)(includes o6 p89)(includes o6 p114)

(waiting o7)
(includes o7 p5)(includes o7 p29)(includes o7 p79)(includes o7 p84)(includes o7 p98)(includes o7 p131)(includes o7 p145)(includes o7 p174)(includes o7 p187)

(waiting o8)
(includes o8 p28)(includes o8 p103)(includes o8 p132)(includes o8 p187)

(waiting o9)
(includes o9 p6)(includes o9 p23)(includes o9 p59)(includes o9 p94)(includes o9 p132)(includes o9 p170)(includes o9 p181)

(waiting o10)
(includes o10 p61)(includes o10 p76)(includes o10 p130)

(waiting o11)
(includes o11 p11)(includes o11 p53)(includes o11 p125)(includes o11 p155)

(waiting o12)
(includes o12 p18)(includes o12 p39)(includes o12 p49)(includes o12 p70)(includes o12 p132)(includes o12 p136)(includes o12 p147)(includes o12 p159)

(waiting o13)
(includes o13 p160)

(waiting o14)
(includes o14 p28)(includes o14 p31)(includes o14 p168)(includes o14 p185)

(waiting o15)
(includes o15 p20)(includes o15 p100)(includes o15 p121)

(waiting o16)
(includes o16 p16)(includes o16 p76)(includes o16 p98)

(waiting o17)
(includes o17 p4)(includes o17 p31)(includes o17 p59)(includes o17 p158)(includes o17 p180)

(waiting o18)
(includes o18 p32)(includes o18 p85)(includes o18 p96)(includes o18 p97)(includes o18 p118)

(waiting o19)
(includes o19 p43)(includes o19 p67)(includes o19 p101)(includes o19 p139)(includes o19 p141)(includes o19 p168)

(waiting o20)
(includes o20 p40)(includes o20 p81)

(waiting o21)
(includes o21 p43)(includes o21 p97)(includes o21 p178)(includes o21 p183)

(waiting o22)
(includes o22 p17)(includes o22 p26)(includes o22 p107)(includes o22 p167)(includes o22 p186)

(waiting o23)
(includes o23 p5)(includes o23 p41)(includes o23 p80)(includes o23 p135)

(waiting o24)
(includes o24 p27)(includes o24 p34)(includes o24 p44)(includes o24 p55)(includes o24 p113)(includes o24 p142)(includes o24 p163)(includes o24 p179)

(waiting o25)
(includes o25 p90)(includes o25 p101)(includes o25 p170)

(waiting o26)
(includes o26 p65)(includes o26 p81)(includes o26 p95)(includes o26 p123)(includes o26 p137)(includes o26 p188)

(waiting o27)
(includes o27 p13)(includes o27 p125)(includes o27 p138)

(waiting o28)
(includes o28 p6)(includes o28 p11)(includes o28 p16)(includes o28 p53)(includes o28 p87)(includes o28 p139)(includes o28 p146)(includes o28 p169)

(waiting o29)
(includes o29 p14)(includes o29 p18)(includes o29 p35)(includes o29 p58)(includes o29 p94)(includes o29 p182)(includes o29 p184)

(waiting o30)
(includes o30 p25)(includes o30 p26)(includes o30 p28)(includes o30 p29)(includes o30 p63)(includes o30 p83)(includes o30 p116)(includes o30 p140)(includes o30 p166)

(waiting o31)
(includes o31 p49)(includes o31 p102)(includes o31 p123)(includes o31 p141)(includes o31 p143)(includes o31 p166)

(waiting o32)
(includes o32 p126)(includes o32 p129)(includes o32 p134)

(waiting o33)
(includes o33 p18)(includes o33 p137)

(waiting o34)
(includes o34 p9)(includes o34 p20)(includes o34 p72)(includes o34 p86)(includes o34 p99)

(waiting o35)
(includes o35 p52)(includes o35 p82)(includes o35 p86)(includes o35 p88)(includes o35 p116)(includes o35 p122)(includes o35 p162)

(waiting o36)
(includes o36 p25)(includes o36 p38)(includes o36 p91)(includes o36 p126)(includes o36 p161)(includes o36 p167)

(waiting o37)
(includes o37 p4)(includes o37 p17)(includes o37 p23)(includes o37 p40)(includes o37 p79)(includes o37 p88)(includes o37 p102)(includes o37 p131)

(waiting o38)
(includes o38 p38)(includes o38 p41)(includes o38 p70)(includes o38 p91)(includes o38 p104)(includes o38 p138)(includes o38 p159)

(waiting o39)
(includes o39 p8)(includes o39 p13)(includes o39 p24)(includes o39 p29)(includes o39 p48)(includes o39 p121)(includes o39 p133)(includes o39 p139)

(waiting o40)
(includes o40 p5)(includes o40 p10)(includes o40 p49)(includes o40 p115)(includes o40 p151)

(waiting o41)
(includes o41 p46)(includes o41 p66)(includes o41 p91)(includes o41 p104)(includes o41 p119)(includes o41 p156)

(waiting o42)
(includes o42 p40)(includes o42 p81)(includes o42 p110)(includes o42 p156)(includes o42 p159)(includes o42 p178)

(waiting o43)
(includes o43 p4)(includes o43 p8)(includes o43 p42)(includes o43 p65)(includes o43 p149)(includes o43 p163)

(waiting o44)
(includes o44 p104)(includes o44 p155)

(waiting o45)
(includes o45 p62)

(waiting o46)
(includes o46 p20)(includes o46 p84)(includes o46 p124)(includes o46 p148)(includes o46 p163)

(waiting o47)
(includes o47 p5)(includes o47 p19)(includes o47 p52)(includes o47 p124)(includes o47 p148)

(waiting o48)
(includes o48 p10)(includes o48 p127)(includes o48 p131)(includes o48 p157)(includes o48 p170)

(waiting o49)
(includes o49 p42)(includes o49 p66)(includes o49 p68)(includes o49 p129)(includes o49 p179)

(waiting o50)
(includes o50 p14)(includes o50 p49)(includes o50 p89)(includes o50 p110)(includes o50 p147)(includes o50 p158)

(waiting o51)
(includes o51 p24)(includes o51 p36)(includes o51 p111)(includes o51 p115)(includes o51 p122)(includes o51 p154)(includes o51 p177)(includes o51 p186)

(waiting o52)
(includes o52 p19)(includes o52 p87)(includes o52 p105)(includes o52 p133)

(waiting o53)
(includes o53 p39)(includes o53 p47)(includes o53 p71)(includes o53 p85)(includes o53 p137)(includes o53 p143)

(waiting o54)
(includes o54 p48)(includes o54 p53)(includes o54 p55)(includes o54 p60)(includes o54 p116)(includes o54 p160)(includes o54 p172)

(waiting o55)
(includes o55 p1)(includes o55 p94)(includes o55 p110)(includes o55 p114)(includes o55 p157)(includes o55 p183)

(waiting o56)
(includes o56 p61)(includes o56 p130)(includes o56 p140)(includes o56 p165)(includes o56 p186)

(waiting o57)
(includes o57 p3)(includes o57 p10)(includes o57 p95)(includes o57 p150)(includes o57 p169)(includes o57 p187)(includes o57 p189)

(waiting o58)
(includes o58 p64)(includes o58 p86)(includes o58 p126)(includes o58 p172)

(waiting o59)
(includes o59 p1)(includes o59 p8)(includes o59 p52)(includes o59 p160)

(waiting o60)
(includes o60 p6)(includes o60 p13)(includes o60 p87)(includes o60 p102)(includes o60 p110)(includes o60 p127)(includes o60 p128)(includes o60 p133)

(waiting o61)
(includes o61 p18)(includes o61 p20)(includes o61 p24)(includes o61 p28)(includes o61 p46)(includes o61 p66)(includes o61 p68)(includes o61 p109)(includes o61 p111)(includes o61 p112)(includes o61 p147)(includes o61 p148)(includes o61 p168)(includes o61 p183)(includes o61 p184)

(waiting o62)
(includes o62 p29)(includes o62 p44)(includes o62 p80)(includes o62 p95)(includes o62 p124)(includes o62 p145)(includes o62 p166)

(waiting o63)
(includes o63 p23)(includes o63 p43)(includes o63 p60)(includes o63 p61)(includes o63 p146)

(waiting o64)
(includes o64 p17)(includes o64 p99)(includes o64 p136)(includes o64 p162)

(waiting o65)
(includes o65 p61)(includes o65 p113)(includes o65 p148)

(waiting o66)
(includes o66 p97)(includes o66 p146)(includes o66 p160)(includes o66 p181)

(waiting o67)
(includes o67 p48)(includes o67 p55)(includes o67 p69)(includes o67 p71)(includes o67 p78)(includes o67 p173)(includes o67 p178)

(waiting o68)
(includes o68 p25)

(waiting o69)
(includes o69 p42)(includes o69 p47)(includes o69 p72)(includes o69 p135)(includes o69 p164)(includes o69 p186)

(waiting o70)
(includes o70 p17)(includes o70 p96)(includes o70 p118)(includes o70 p144)(includes o70 p157)(includes o70 p168)(includes o70 p172)(includes o70 p175)

(waiting o71)
(includes o71 p13)(includes o71 p17)(includes o71 p32)(includes o71 p56)(includes o71 p66)(includes o71 p78)(includes o71 p102)(includes o71 p107)(includes o71 p179)

(waiting o72)
(includes o72 p8)(includes o72 p17)(includes o72 p40)(includes o72 p96)(includes o72 p135)(includes o72 p149)(includes o72 p164)(includes o72 p169)

(waiting o73)
(includes o73 p16)(includes o73 p19)(includes o73 p38)(includes o73 p97)(includes o73 p116)(includes o73 p132)

(waiting o74)
(includes o74 p10)(includes o74 p28)(includes o74 p105)(includes o74 p110)(includes o74 p116)(includes o74 p133)(includes o74 p138)

(waiting o75)
(includes o75 p13)(includes o75 p91)(includes o75 p118)(includes o75 p124)(includes o75 p125)(includes o75 p172)(includes o75 p182)

(waiting o76)
(includes o76 p56)(includes o76 p157)(includes o76 p161)

(waiting o77)
(includes o77 p39)(includes o77 p65)(includes o77 p142)(includes o77 p165)(includes o77 p178)(includes o77 p181)

(waiting o78)
(includes o78 p2)(includes o78 p5)(includes o78 p69)(includes o78 p77)(includes o78 p104)(includes o78 p108)(includes o78 p110)(includes o78 p127)(includes o78 p132)(includes o78 p163)

(waiting o79)
(includes o79 p62)(includes o79 p123)(includes o79 p157)

(waiting o80)
(includes o80 p34)(includes o80 p36)(includes o80 p43)(includes o80 p78)(includes o80 p109)(includes o80 p116)(includes o80 p153)

(waiting o81)
(includes o81 p5)(includes o81 p62)(includes o81 p95)(includes o81 p189)(includes o81 p190)

(waiting o82)
(includes o82 p41)(includes o82 p130)(includes o82 p147)(includes o82 p165)(includes o82 p173)

(waiting o83)
(includes o83 p56)(includes o83 p63)

(waiting o84)
(includes o84 p25)(includes o84 p71)(includes o84 p73)(includes o84 p74)(includes o84 p84)(includes o84 p92)(includes o84 p95)(includes o84 p168)(includes o84 p176)(includes o84 p185)(includes o84 p186)

(waiting o85)
(includes o85 p49)(includes o85 p129)(includes o85 p150)(includes o85 p153)

(waiting o86)
(includes o86 p9)(includes o86 p20)(includes o86 p25)(includes o86 p36)(includes o86 p98)(includes o86 p108)(includes o86 p109)(includes o86 p116)(includes o86 p140)

(waiting o87)
(includes o87 p7)(includes o87 p36)(includes o87 p41)(includes o87 p97)(includes o87 p151)(includes o87 p161)

(waiting o88)
(includes o88 p11)(includes o88 p20)(includes o88 p50)(includes o88 p98)(includes o88 p119)(includes o88 p121)(includes o88 p145)(includes o88 p153)(includes o88 p174)

(waiting o89)
(includes o89 p19)(includes o89 p20)(includes o89 p41)(includes o89 p68)(includes o89 p83)(includes o89 p126)(includes o89 p187)

(waiting o90)
(includes o90 p34)

(waiting o91)
(includes o91 p3)(includes o91 p9)(includes o91 p154)

(waiting o92)
(includes o92 p4)(includes o92 p142)(includes o92 p173)

(waiting o93)
(includes o93 p66)(includes o93 p71)(includes o93 p118)

(waiting o94)
(includes o94 p21)(includes o94 p40)(includes o94 p56)(includes o94 p109)(includes o94 p123)(includes o94 p147)(includes o94 p173)

(waiting o95)
(includes o95 p5)(includes o95 p26)(includes o95 p98)(includes o95 p190)

(waiting o96)
(includes o96 p25)(includes o96 p29)(includes o96 p38)(includes o96 p48)(includes o96 p66)(includes o96 p68)(includes o96 p103)(includes o96 p106)(includes o96 p135)(includes o96 p157)

(waiting o97)
(includes o97 p17)(includes o97 p51)(includes o97 p130)(includes o97 p134)(includes o97 p173)

(waiting o98)
(includes o98 p20)(includes o98 p27)(includes o98 p45)(includes o98 p71)(includes o98 p131)(includes o98 p160)

(waiting o99)
(includes o99 p104)(includes o99 p146)(includes o99 p151)

(waiting o100)
(includes o100 p31)(includes o100 p70)(includes o100 p130)(includes o100 p171)(includes o100 p172)

(waiting o101)
(includes o101 p76)(includes o101 p96)(includes o101 p108)(includes o101 p126)(includes o101 p139)(includes o101 p166)

(waiting o102)
(includes o102 p53)(includes o102 p82)(includes o102 p87)(includes o102 p140)(includes o102 p162)

(waiting o103)
(includes o103 p6)(includes o103 p66)(includes o103 p85)(includes o103 p88)(includes o103 p111)

(waiting o104)
(includes o104 p40)(includes o104 p79)(includes o104 p113)

(waiting o105)
(includes o105 p6)(includes o105 p25)(includes o105 p26)(includes o105 p104)(includes o105 p116)(includes o105 p124)(includes o105 p150)

(waiting o106)
(includes o106 p38)(includes o106 p55)(includes o106 p137)(includes o106 p144)(includes o106 p147)(includes o106 p190)

(waiting o107)
(includes o107 p101)

(waiting o108)
(includes o108 p71)(includes o108 p103)(includes o108 p114)(includes o108 p172)

(waiting o109)
(includes o109 p29)(includes o109 p42)(includes o109 p70)(includes o109 p154)(includes o109 p155)(includes o109 p183)

(waiting o110)
(includes o110 p9)(includes o110 p46)(includes o110 p129)(includes o110 p157)

(waiting o111)
(includes o111 p5)(includes o111 p8)(includes o111 p13)(includes o111 p34)(includes o111 p79)(includes o111 p89)(includes o111 p103)(includes o111 p172)(includes o111 p180)

(waiting o112)
(includes o112 p30)(includes o112 p62)(includes o112 p76)(includes o112 p95)(includes o112 p97)(includes o112 p105)(includes o112 p121)(includes o112 p127)(includes o112 p180)(includes o112 p187)

(waiting o113)
(includes o113 p4)(includes o113 p87)(includes o113 p109)(includes o113 p118)

(waiting o114)
(includes o114 p21)(includes o114 p40)(includes o114 p170)

(waiting o115)
(includes o115 p45)(includes o115 p107)(includes o115 p109)(includes o115 p121)(includes o115 p122)(includes o115 p156)

(waiting o116)
(includes o116 p42)(includes o116 p47)(includes o116 p89)(includes o116 p178)

(waiting o117)
(includes o117 p8)(includes o117 p92)(includes o117 p113)(includes o117 p139)

(waiting o118)
(includes o118 p64)(includes o118 p70)(includes o118 p96)(includes o118 p111)(includes o118 p153)

(waiting o119)
(includes o119 p91)(includes o119 p97)(includes o119 p124)(includes o119 p186)(includes o119 p189)

(waiting o120)
(includes o120 p46)(includes o120 p94)(includes o120 p112)(includes o120 p130)(includes o120 p168)

(waiting o121)
(includes o121 p46)(includes o121 p57)(includes o121 p100)(includes o121 p108)(includes o121 p132)(includes o121 p147)(includes o121 p148)(includes o121 p173)(includes o121 p180)

(waiting o122)
(includes o122 p34)(includes o122 p56)(includes o122 p88)(includes o122 p89)(includes o122 p130)(includes o122 p135)(includes o122 p142)(includes o122 p160)

(waiting o123)
(includes o123 p34)(includes o123 p40)(includes o123 p48)(includes o123 p55)(includes o123 p77)(includes o123 p81)(includes o123 p94)(includes o123 p172)

(waiting o124)
(includes o124 p78)(includes o124 p137)(includes o124 p144)(includes o124 p152)(includes o124 p168)

(waiting o125)
(includes o125 p59)(includes o125 p91)(includes o125 p115)(includes o125 p135)(includes o125 p153)(includes o125 p180)

(waiting o126)
(includes o126 p7)(includes o126 p27)(includes o126 p90)(includes o126 p99)(includes o126 p129)(includes o126 p151)(includes o126 p155)(includes o126 p156)(includes o126 p164)

(waiting o127)
(includes o127 p14)(includes o127 p46)(includes o127 p68)(includes o127 p100)

(waiting o128)
(includes o128 p2)(includes o128 p146)(includes o128 p155)

(waiting o129)
(includes o129 p77)(includes o129 p103)(includes o129 p107)(includes o129 p116)(includes o129 p132)(includes o129 p156)(includes o129 p159)

(waiting o130)
(includes o130 p41)(includes o130 p48)(includes o130 p57)(includes o130 p59)(includes o130 p81)(includes o130 p88)(includes o130 p122)(includes o130 p180)

(waiting o131)
(includes o131 p13)(includes o131 p51)(includes o131 p57)(includes o131 p68)(includes o131 p76)(includes o131 p88)(includes o131 p129)(includes o131 p156)

(waiting o132)
(includes o132 p33)(includes o132 p38)(includes o132 p148)(includes o132 p151)

(waiting o133)
(includes o133 p54)(includes o133 p64)(includes o133 p77)(includes o133 p138)(includes o133 p141)(includes o133 p149)(includes o133 p158)(includes o133 p176)

(waiting o134)
(includes o134 p75)(includes o134 p85)

(waiting o135)
(includes o135 p23)(includes o135 p54)(includes o135 p60)(includes o135 p102)(includes o135 p128)

(waiting o136)
(includes o136 p56)(includes o136 p117)(includes o136 p131)(includes o136 p135)(includes o136 p179)

(waiting o137)
(includes o137 p21)(includes o137 p56)(includes o137 p81)(includes o137 p171)

(waiting o138)
(includes o138 p25)(includes o138 p62)(includes o138 p122)(includes o138 p127)(includes o138 p157)(includes o138 p184)

(waiting o139)
(includes o139 p14)(includes o139 p40)(includes o139 p68)(includes o139 p75)(includes o139 p79)(includes o139 p123)

(waiting o140)
(includes o140 p42)(includes o140 p138)(includes o140 p164)

(waiting o141)
(includes o141 p48)(includes o141 p60)(includes o141 p91)(includes o141 p96)(includes o141 p146)(includes o141 p155)

(waiting o142)
(includes o142 p14)(includes o142 p67)(includes o142 p70)(includes o142 p78)(includes o142 p138)

(waiting o143)
(includes o143 p12)(includes o143 p37)(includes o143 p84)(includes o143 p131)

(waiting o144)
(includes o144 p33)(includes o144 p46)(includes o144 p88)(includes o144 p114)(includes o144 p127)(includes o144 p133)(includes o144 p143)(includes o144 p171)(includes o144 p174)(includes o144 p178)(includes o144 p183)

(waiting o145)
(includes o145 p36)(includes o145 p78)(includes o145 p112)(includes o145 p115)(includes o145 p178)

(waiting o146)
(includes o146 p22)(includes o146 p36)(includes o146 p66)(includes o146 p173)(includes o146 p181)

(waiting o147)
(includes o147 p98)(includes o147 p108)(includes o147 p174)

(waiting o148)
(includes o148 p6)(includes o148 p10)(includes o148 p74)(includes o148 p91)(includes o148 p106)(includes o148 p183)

(waiting o149)
(includes o149 p36)(includes o149 p93)(includes o149 p120)(includes o149 p134)(includes o149 p172)

(waiting o150)
(includes o150 p69)(includes o150 p82)(includes o150 p111)

(waiting o151)
(includes o151 p36)(includes o151 p82)(includes o151 p147)(includes o151 p153)(includes o151 p183)

(waiting o152)
(includes o152 p20)(includes o152 p38)(includes o152 p47)(includes o152 p73)(includes o152 p76)(includes o152 p106)(includes o152 p179)

(waiting o153)
(includes o153 p70)(includes o153 p90)(includes o153 p130)(includes o153 p134)(includes o153 p141)

(waiting o154)
(includes o154 p40)(includes o154 p127)(includes o154 p131)(includes o154 p147)(includes o154 p180)(includes o154 p183)(includes o154 p186)

(waiting o155)
(includes o155 p27)(includes o155 p75)(includes o155 p94)(includes o155 p157)

(waiting o156)
(includes o156 p6)(includes o156 p52)(includes o156 p73)(includes o156 p86)(includes o156 p163)

(waiting o157)
(includes o157 p105)(includes o157 p162)(includes o157 p190)

(waiting o158)
(includes o158 p11)(includes o158 p85)(includes o158 p187)(includes o158 p190)

(waiting o159)
(includes o159 p7)(includes o159 p54)(includes o159 p149)

(waiting o160)
(includes o160 p43)(includes o160 p81)(includes o160 p116)(includes o160 p130)(includes o160 p149)

(waiting o161)
(includes o161 p120)(includes o161 p148)(includes o161 p190)

(waiting o162)
(includes o162 p13)(includes o162 p37)(includes o162 p70)(includes o162 p73)(includes o162 p121)(includes o162 p145)

(waiting o163)
(includes o163 p97)(includes o163 p130)(includes o163 p189)

(waiting o164)
(includes o164 p2)(includes o164 p80)(includes o164 p81)(includes o164 p92)(includes o164 p93)(includes o164 p104)(includes o164 p136)(includes o164 p142)(includes o164 p145)

(waiting o165)
(includes o165 p150)(includes o165 p182)

(waiting o166)
(includes o166 p15)(includes o166 p44)(includes o166 p49)(includes o166 p140)(includes o166 p166)(includes o166 p190)

(waiting o167)
(includes o167 p6)(includes o167 p84)(includes o167 p96)(includes o167 p157)(includes o167 p164)(includes o167 p175)

(waiting o168)
(includes o168 p2)(includes o168 p8)(includes o168 p31)(includes o168 p51)(includes o168 p58)(includes o168 p69)(includes o168 p176)

(waiting o169)
(includes o169 p115)(includes o169 p151)(includes o169 p186)

(waiting o170)
(includes o170 p56)(includes o170 p59)(includes o170 p91)(includes o170 p149)(includes o170 p164)

(waiting o171)
(includes o171 p10)(includes o171 p37)(includes o171 p67)(includes o171 p69)(includes o171 p127)(includes o171 p180)

(waiting o172)
(includes o172 p33)(includes o172 p51)(includes o172 p79)(includes o172 p93)(includes o172 p99)

(waiting o173)
(includes o173 p5)(includes o173 p38)(includes o173 p51)(includes o173 p68)(includes o173 p74)(includes o173 p83)(includes o173 p95)(includes o173 p157)(includes o173 p161)

(waiting o174)
(includes o174 p89)(includes o174 p125)(includes o174 p135)(includes o174 p188)

(waiting o175)
(includes o175 p52)(includes o175 p60)(includes o175 p74)(includes o175 p84)(includes o175 p169)(includes o175 p176)

(waiting o176)
(includes o176 p32)(includes o176 p73)(includes o176 p108)(includes o176 p116)(includes o176 p157)

(waiting o177)
(includes o177 p4)(includes o177 p28)(includes o177 p47)(includes o177 p153)(includes o177 p170)(includes o177 p183)

(waiting o178)
(includes o178 p66)(includes o178 p100)(includes o178 p102)

(waiting o179)
(includes o179 p6)(includes o179 p35)(includes o179 p87)(includes o179 p102)(includes o179 p110)(includes o179 p173)

(waiting o180)
(includes o180 p21)(includes o180 p57)(includes o180 p61)(includes o180 p94)(includes o180 p110)(includes o180 p132)

(waiting o181)
(includes o181 p52)(includes o181 p53)(includes o181 p92)(includes o181 p136)(includes o181 p153)

(waiting o182)
(includes o182 p2)(includes o182 p79)(includes o182 p88)(includes o182 p148)(includes o182 p190)

(waiting o183)
(includes o183 p33)(includes o183 p63)(includes o183 p65)(includes o183 p118)

(waiting o184)
(includes o184 p45)(includes o184 p133)(includes o184 p149)

(waiting o185)
(includes o185 p29)(includes o185 p129)(includes o185 p159)(includes o185 p173)(includes o185 p179)

(waiting o186)
(includes o186 p71)(includes o186 p120)(includes o186 p161)

(waiting o187)
(includes o187 p70)(includes o187 p87)(includes o187 p97)

(waiting o188)
(includes o188 p24)(includes o188 p158)

(waiting o189)
(includes o189 p21)(includes o189 p22)(includes o189 p60)(includes o189 p102)(includes o189 p135)(includes o189 p158)

(waiting o190)
(includes o190 p3)(includes o190 p39)(includes o190 p40)(includes o190 p57)

(not-made p1)
(not-made p2)
(not-made p3)
(not-made p4)
(not-made p5)
(not-made p6)
(not-made p7)
(not-made p8)
(not-made p9)
(not-made p10)
(not-made p11)
(not-made p12)
(not-made p13)
(not-made p14)
(not-made p15)
(not-made p16)
(not-made p17)
(not-made p18)
(not-made p19)
(not-made p20)
(not-made p21)
(not-made p22)
(not-made p23)
(not-made p24)
(not-made p25)
(not-made p26)
(not-made p27)
(not-made p28)
(not-made p29)
(not-made p30)
(not-made p31)
(not-made p32)
(not-made p33)
(not-made p34)
(not-made p35)
(not-made p36)
(not-made p37)
(not-made p38)
(not-made p39)
(not-made p40)
(not-made p41)
(not-made p42)
(not-made p43)
(not-made p44)
(not-made p45)
(not-made p46)
(not-made p47)
(not-made p48)
(not-made p49)
(not-made p50)
(not-made p51)
(not-made p52)
(not-made p53)
(not-made p54)
(not-made p55)
(not-made p56)
(not-made p57)
(not-made p58)
(not-made p59)
(not-made p60)
(not-made p61)
(not-made p62)
(not-made p63)
(not-made p64)
(not-made p65)
(not-made p66)
(not-made p67)
(not-made p68)
(not-made p69)
(not-made p70)
(not-made p71)
(not-made p72)
(not-made p73)
(not-made p74)
(not-made p75)
(not-made p76)
(not-made p77)
(not-made p78)
(not-made p79)
(not-made p80)
(not-made p81)
(not-made p82)
(not-made p83)
(not-made p84)
(not-made p85)
(not-made p86)
(not-made p87)
(not-made p88)
(not-made p89)
(not-made p90)
(not-made p91)
(not-made p92)
(not-made p93)
(not-made p94)
(not-made p95)
(not-made p96)
(not-made p97)
(not-made p98)
(not-made p99)
(not-made p100)
(not-made p101)
(not-made p102)
(not-made p103)
(not-made p104)
(not-made p105)
(not-made p106)
(not-made p107)
(not-made p108)
(not-made p109)
(not-made p110)
(not-made p111)
(not-made p112)
(not-made p113)
(not-made p114)
(not-made p115)
(not-made p116)
(not-made p117)
(not-made p118)
(not-made p119)
(not-made p120)
(not-made p121)
(not-made p122)
(not-made p123)
(not-made p124)
(not-made p125)
(not-made p126)
(not-made p127)
(not-made p128)
(not-made p129)
(not-made p130)
(not-made p131)
(not-made p132)
(not-made p133)
(not-made p134)
(not-made p135)
(not-made p136)
(not-made p137)
(not-made p138)
(not-made p139)
(not-made p140)
(not-made p141)
(not-made p142)
(not-made p143)
(not-made p144)
(not-made p145)
(not-made p146)
(not-made p147)
(not-made p148)
(not-made p149)
(not-made p150)
(not-made p151)
(not-made p152)
(not-made p153)
(not-made p154)
(not-made p155)
(not-made p156)
(not-made p157)
(not-made p158)
(not-made p159)
(not-made p160)
(not-made p161)
(not-made p162)
(not-made p163)
(not-made p164)
(not-made p165)
(not-made p166)
(not-made p167)
(not-made p168)
(not-made p169)
(not-made p170)
(not-made p171)
(not-made p172)
(not-made p173)
(not-made p174)
(not-made p175)
(not-made p176)
(not-made p177)
(not-made p178)
(not-made p179)
(not-made p180)
(not-made p181)
(not-made p182)
(not-made p183)
(not-made p184)
(not-made p185)
(not-made p186)
(not-made p187)
(not-made p188)
(not-made p189)
(not-made p190)

(= (total-cost) 0)

)

(:goal
(and
(shipped o1)
(shipped o2)
(shipped o3)
(shipped o4)
(shipped o5)
(shipped o6)
(shipped o7)
(shipped o8)
(shipped o9)
(shipped o10)
(shipped o11)
(shipped o12)
(shipped o13)
(shipped o14)
(shipped o15)
(shipped o16)
(shipped o17)
(shipped o18)
(shipped o19)
(shipped o20)
(shipped o21)
(shipped o22)
(shipped o23)
(shipped o24)
(shipped o25)
(shipped o26)
(shipped o27)
(shipped o28)
(shipped o29)
(shipped o30)
(shipped o31)
(shipped o32)
(shipped o33)
(shipped o34)
(shipped o35)
(shipped o36)
(shipped o37)
(shipped o38)
(shipped o39)
(shipped o40)
(shipped o41)
(shipped o42)
(shipped o43)
(shipped o44)
(shipped o45)
(shipped o46)
(shipped o47)
(shipped o48)
(shipped o49)
(shipped o50)
(shipped o51)
(shipped o52)
(shipped o53)
(shipped o54)
(shipped o55)
(shipped o56)
(shipped o57)
(shipped o58)
(shipped o59)
(shipped o60)
(shipped o61)
(shipped o62)
(shipped o63)
(shipped o64)
(shipped o65)
(shipped o66)
(shipped o67)
(shipped o68)
(shipped o69)
(shipped o70)
(shipped o71)
(shipped o72)
(shipped o73)
(shipped o74)
(shipped o75)
(shipped o76)
(shipped o77)
(shipped o78)
(shipped o79)
(shipped o80)
(shipped o81)
(shipped o82)
(shipped o83)
(shipped o84)
(shipped o85)
(shipped o86)
(shipped o87)
(shipped o88)
(shipped o89)
(shipped o90)
(shipped o91)
(shipped o92)
(shipped o93)
(shipped o94)
(shipped o95)
(shipped o96)
(shipped o97)
(shipped o98)
(shipped o99)
(shipped o100)
(shipped o101)
(shipped o102)
(shipped o103)
(shipped o104)
(shipped o105)
(shipped o106)
(shipped o107)
(shipped o108)
(shipped o109)
(shipped o110)
(shipped o111)
(shipped o112)
(shipped o113)
(shipped o114)
(shipped o115)
(shipped o116)
(shipped o117)
(shipped o118)
(shipped o119)
(shipped o120)
(shipped o121)
(shipped o122)
(shipped o123)
(shipped o124)
(shipped o125)
(shipped o126)
(shipped o127)
(shipped o128)
(shipped o129)
(shipped o130)
(shipped o131)
(shipped o132)
(shipped o133)
(shipped o134)
(shipped o135)
(shipped o136)
(shipped o137)
(shipped o138)
(shipped o139)
(shipped o140)
(shipped o141)
(shipped o142)
(shipped o143)
(shipped o144)
(shipped o145)
(shipped o146)
(shipped o147)
(shipped o148)
(shipped o149)
(shipped o150)
(shipped o151)
(shipped o152)
(shipped o153)
(shipped o154)
(shipped o155)
(shipped o156)
(shipped o157)
(shipped o158)
(shipped o159)
(shipped o160)
(shipped o161)
(shipped o162)
(shipped o163)
(shipped o164)
(shipped o165)
(shipped o166)
(shipped o167)
(shipped o168)
(shipped o169)
(shipped o170)
(shipped o171)
(shipped o172)
(shipped o173)
(shipped o174)
(shipped o175)
(shipped o176)
(shipped o177)
(shipped o178)
(shipped o179)
(shipped o180)
(shipped o181)
(shipped o182)
(shipped o183)
(shipped o184)
(shipped o185)
(shipped o186)
(shipped o187)
(shipped o188)
(shipped o189)
(shipped o190)
))

(:metric minimize (total-cost))

)


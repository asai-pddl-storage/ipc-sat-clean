(define (problem os-sequencedstrips-p230_3)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) 
(stacks-avail n0)

(waiting o1)
(includes o1 p26)(includes o1 p50)(includes o1 p96)(includes o1 p104)(includes o1 p110)(includes o1 p157)(includes o1 p173)(includes o1 p194)

(waiting o2)
(includes o2 p2)(includes o2 p18)(includes o2 p28)(includes o2 p33)(includes o2 p68)(includes o2 p94)(includes o2 p98)(includes o2 p183)(includes o2 p185)

(waiting o3)
(includes o3 p66)(includes o3 p142)(includes o3 p222)

(waiting o4)
(includes o4 p8)(includes o4 p56)(includes o4 p74)(includes o4 p141)(includes o4 p149)(includes o4 p163)

(waiting o5)
(includes o5 p16)(includes o5 p50)(includes o5 p113)(includes o5 p143)(includes o5 p175)(includes o5 p195)

(waiting o6)
(includes o6 p49)(includes o6 p66)(includes o6 p81)(includes o6 p95)(includes o6 p139)(includes o6 p158)(includes o6 p190)(includes o6 p220)

(waiting o7)
(includes o7 p50)(includes o7 p69)(includes o7 p80)(includes o7 p85)(includes o7 p98)

(waiting o8)
(includes o8 p20)(includes o8 p32)(includes o8 p82)(includes o8 p120)(includes o8 p128)(includes o8 p133)(includes o8 p135)(includes o8 p142)(includes o8 p150)(includes o8 p158)(includes o8 p165)

(waiting o9)
(includes o9 p76)(includes o9 p171)(includes o9 p175)(includes o9 p180)(includes o9 p190)(includes o9 p197)(includes o9 p209)

(waiting o10)
(includes o10 p6)(includes o10 p36)(includes o10 p41)(includes o10 p55)(includes o10 p57)(includes o10 p120)(includes o10 p122)(includes o10 p137)(includes o10 p181)

(waiting o11)
(includes o11 p23)(includes o11 p201)(includes o11 p212)

(waiting o12)
(includes o12 p56)(includes o12 p94)(includes o12 p194)(includes o12 p209)(includes o12 p223)

(waiting o13)
(includes o13 p7)(includes o13 p55)(includes o13 p100)(includes o13 p184)

(waiting o14)
(includes o14 p2)(includes o14 p43)(includes o14 p78)(includes o14 p113)(includes o14 p144)(includes o14 p171)(includes o14 p220)

(waiting o15)
(includes o15 p73)(includes o15 p168)

(waiting o16)
(includes o16 p9)(includes o16 p43)(includes o16 p60)(includes o16 p61)(includes o16 p62)(includes o16 p125)(includes o16 p134)(includes o16 p140)(includes o16 p166)(includes o16 p180)(includes o16 p219)

(waiting o17)
(includes o17 p18)(includes o17 p34)(includes o17 p131)(includes o17 p164)(includes o17 p197)

(waiting o18)
(includes o18 p19)(includes o18 p43)(includes o18 p44)(includes o18 p61)(includes o18 p70)(includes o18 p101)(includes o18 p108)

(waiting o19)
(includes o19 p24)(includes o19 p35)(includes o19 p41)(includes o19 p84)(includes o19 p156)(includes o19 p176)(includes o19 p227)

(waiting o20)
(includes o20 p1)(includes o20 p51)(includes o20 p146)(includes o20 p179)

(waiting o21)
(includes o21 p35)(includes o21 p41)(includes o21 p50)(includes o21 p85)(includes o21 p98)(includes o21 p131)(includes o21 p230)

(waiting o22)
(includes o22 p15)(includes o22 p41)(includes o22 p112)(includes o22 p126)(includes o22 p145)(includes o22 p165)(includes o22 p187)

(waiting o23)
(includes o23 p35)(includes o23 p73)(includes o23 p79)(includes o23 p80)(includes o23 p102)(includes o23 p104)

(waiting o24)
(includes o24 p51)(includes o24 p53)(includes o24 p86)(includes o24 p95)(includes o24 p174)(includes o24 p209)

(waiting o25)
(includes o25 p11)(includes o25 p21)(includes o25 p32)(includes o25 p44)(includes o25 p47)(includes o25 p90)(includes o25 p134)(includes o25 p211)(includes o25 p216)

(waiting o26)
(includes o26 p26)(includes o26 p28)(includes o26 p67)(includes o26 p80)(includes o26 p82)(includes o26 p88)(includes o26 p92)(includes o26 p100)(includes o26 p126)(includes o26 p147)(includes o26 p148)(includes o26 p183)(includes o26 p212)

(waiting o27)
(includes o27 p35)(includes o27 p41)(includes o27 p164)(includes o27 p221)

(waiting o28)
(includes o28 p74)(includes o28 p110)(includes o28 p157)(includes o28 p172)

(waiting o29)
(includes o29 p108)(includes o29 p139)(includes o29 p198)

(waiting o30)
(includes o30 p1)(includes o30 p50)(includes o30 p147)(includes o30 p188)(includes o30 p192)(includes o30 p202)(includes o30 p206)(includes o30 p215)

(waiting o31)
(includes o31 p64)(includes o31 p138)(includes o31 p198)(includes o31 p204)

(waiting o32)
(includes o32 p41)(includes o32 p63)(includes o32 p122)(includes o32 p135)(includes o32 p160)(includes o32 p173)(includes o32 p220)(includes o32 p225)(includes o32 p226)

(waiting o33)
(includes o33 p3)(includes o33 p10)(includes o33 p73)(includes o33 p129)(includes o33 p184)(includes o33 p220)

(waiting o34)
(includes o34 p47)(includes o34 p185)

(waiting o35)
(includes o35 p15)(includes o35 p200)

(waiting o36)
(includes o36 p25)(includes o36 p30)(includes o36 p95)(includes o36 p120)(includes o36 p188)(includes o36 p200)(includes o36 p223)

(waiting o37)
(includes o37 p12)(includes o37 p44)(includes o37 p71)(includes o37 p124)(includes o37 p140)(includes o37 p159)

(waiting o38)
(includes o38 p39)(includes o38 p75)(includes o38 p100)(includes o38 p172)(includes o38 p214)(includes o38 p216)

(waiting o39)
(includes o39 p24)(includes o39 p46)(includes o39 p54)(includes o39 p64)(includes o39 p68)(includes o39 p90)

(waiting o40)
(includes o40 p56)(includes o40 p61)(includes o40 p92)(includes o40 p121)(includes o40 p156)(includes o40 p165)(includes o40 p174)

(waiting o41)
(includes o41 p55)(includes o41 p58)(includes o41 p76)(includes o41 p87)(includes o41 p95)(includes o41 p99)(includes o41 p107)(includes o41 p164)(includes o41 p184)(includes o41 p194)

(waiting o42)
(includes o42 p32)(includes o42 p143)(includes o42 p151)(includes o42 p166)(includes o42 p178)(includes o42 p225)

(waiting o43)
(includes o43 p32)(includes o43 p118)(includes o43 p158)(includes o43 p161)(includes o43 p163)(includes o43 p186)

(waiting o44)
(includes o44 p48)(includes o44 p121)(includes o44 p146)(includes o44 p152)(includes o44 p155)(includes o44 p158)(includes o44 p226)

(waiting o45)
(includes o45 p1)(includes o45 p40)(includes o45 p121)(includes o45 p225)

(waiting o46)
(includes o46 p7)(includes o46 p9)(includes o46 p108)(includes o46 p110)(includes o46 p207)(includes o46 p220)

(waiting o47)
(includes o47 p58)(includes o47 p87)

(waiting o48)
(includes o48 p72)(includes o48 p148)(includes o48 p156)

(waiting o49)
(includes o49 p10)(includes o49 p13)(includes o49 p27)(includes o49 p68)(includes o49 p125)(includes o49 p180)(includes o49 p210)(includes o49 p211)

(waiting o50)
(includes o50 p5)(includes o50 p11)(includes o50 p16)(includes o50 p24)(includes o50 p106)(includes o50 p117)(includes o50 p149)(includes o50 p176)(includes o50 p196)(includes o50 p216)(includes o50 p225)

(waiting o51)
(includes o51 p80)(includes o51 p131)(includes o51 p133)(includes o51 p136)(includes o51 p150)(includes o51 p161)(includes o51 p199)

(waiting o52)
(includes o52 p40)(includes o52 p120)(includes o52 p149)(includes o52 p170)(includes o52 p183)

(waiting o53)
(includes o53 p23)(includes o53 p24)(includes o53 p32)(includes o53 p104)(includes o53 p137)(includes o53 p159)(includes o53 p210)(includes o53 p223)

(waiting o54)
(includes o54 p1)(includes o54 p52)(includes o54 p58)(includes o54 p65)(includes o54 p66)(includes o54 p76)(includes o54 p99)(includes o54 p108)(includes o54 p170)(includes o54 p208)

(waiting o55)
(includes o55 p5)(includes o55 p32)(includes o55 p33)(includes o55 p96)(includes o55 p166)(includes o55 p169)(includes o55 p171)(includes o55 p179)(includes o55 p199)

(waiting o56)
(includes o56 p36)(includes o56 p51)(includes o56 p78)(includes o56 p84)(includes o56 p121)(includes o56 p212)

(waiting o57)
(includes o57 p47)(includes o57 p80)(includes o57 p87)(includes o57 p180)(includes o57 p216)

(waiting o58)
(includes o58 p3)(includes o58 p18)(includes o58 p37)(includes o58 p44)(includes o58 p52)(includes o58 p114)(includes o58 p116)(includes o58 p215)(includes o58 p221)

(waiting o59)
(includes o59 p4)(includes o59 p7)(includes o59 p11)(includes o59 p18)(includes o59 p110)(includes o59 p117)(includes o59 p136)(includes o59 p165)(includes o59 p189)(includes o59 p222)

(waiting o60)
(includes o60 p62)(includes o60 p115)(includes o60 p123)(includes o60 p162)(includes o60 p170)(includes o60 p181)(includes o60 p223)(includes o60 p224)

(waiting o61)
(includes o61 p30)(includes o61 p52)(includes o61 p127)(includes o61 p130)(includes o61 p138)(includes o61 p169)(includes o61 p182)

(waiting o62)
(includes o62 p21)(includes o62 p56)(includes o62 p91)(includes o62 p134)(includes o62 p166)(includes o62 p180)(includes o62 p185)(includes o62 p189)(includes o62 p205)

(waiting o63)
(includes o63 p35)(includes o63 p85)(includes o63 p110)(includes o63 p126)(includes o63 p147)(includes o63 p191)

(waiting o64)
(includes o64 p16)

(waiting o65)
(includes o65 p39)(includes o65 p139)(includes o65 p170)(includes o65 p189)

(waiting o66)
(includes o66 p118)(includes o66 p139)(includes o66 p152)(includes o66 p173)(includes o66 p187)(includes o66 p222)

(waiting o67)
(includes o67 p42)(includes o67 p87)(includes o67 p101)(includes o67 p126)(includes o67 p142)(includes o67 p152)(includes o67 p178)

(waiting o68)
(includes o68 p1)(includes o68 p90)(includes o68 p104)(includes o68 p122)(includes o68 p164)

(waiting o69)
(includes o69 p73)(includes o69 p118)

(waiting o70)
(includes o70 p17)(includes o70 p29)(includes o70 p105)(includes o70 p206)

(waiting o71)
(includes o71 p74)(includes o71 p144)(includes o71 p158)

(waiting o72)
(includes o72 p52)(includes o72 p56)(includes o72 p59)(includes o72 p67)(includes o72 p147)(includes o72 p229)

(waiting o73)
(includes o73 p216)

(waiting o74)
(includes o74 p4)(includes o74 p75)(includes o74 p105)(includes o74 p149)(includes o74 p174)(includes o74 p209)

(waiting o75)
(includes o75 p39)(includes o75 p53)(includes o75 p60)(includes o75 p73)(includes o75 p96)(includes o75 p103)(includes o75 p114)(includes o75 p128)(includes o75 p150)(includes o75 p179)

(waiting o76)
(includes o76 p28)(includes o76 p49)(includes o76 p66)(includes o76 p111)(includes o76 p139)(includes o76 p159)(includes o76 p167)

(waiting o77)
(includes o77 p36)(includes o77 p41)(includes o77 p71)(includes o77 p83)(includes o77 p169)(includes o77 p219)(includes o77 p227)

(waiting o78)
(includes o78 p9)(includes o78 p44)(includes o78 p68)(includes o78 p78)(includes o78 p167)(includes o78 p179)(includes o78 p219)

(waiting o79)
(includes o79 p50)(includes o79 p75)(includes o79 p216)

(waiting o80)
(includes o80 p49)(includes o80 p201)

(waiting o81)
(includes o81 p19)(includes o81 p26)(includes o81 p29)(includes o81 p32)(includes o81 p46)(includes o81 p123)(includes o81 p163)

(waiting o82)
(includes o82 p7)(includes o82 p11)(includes o82 p129)(includes o82 p136)(includes o82 p140)(includes o82 p161)(includes o82 p191)(includes o82 p224)

(waiting o83)
(includes o83 p33)(includes o83 p65)(includes o83 p88)(includes o83 p140)(includes o83 p150)(includes o83 p205)

(waiting o84)
(includes o84 p176)

(waiting o85)
(includes o85 p21)(includes o85 p31)(includes o85 p127)

(waiting o86)
(includes o86 p70)(includes o86 p84)(includes o86 p116)(includes o86 p154)(includes o86 p204)(includes o86 p216)(includes o86 p217)

(waiting o87)
(includes o87 p25)(includes o87 p121)(includes o87 p143)(includes o87 p161)(includes o87 p178)(includes o87 p189)

(waiting o88)
(includes o88 p207)(includes o88 p222)(includes o88 p223)

(waiting o89)
(includes o89 p7)(includes o89 p97)(includes o89 p208)

(waiting o90)
(includes o90 p5)(includes o90 p65)(includes o90 p83)(includes o90 p91)(includes o90 p113)(includes o90 p161)(includes o90 p188)(includes o90 p218)

(waiting o91)
(includes o91 p3)(includes o91 p45)(includes o91 p76)(includes o91 p82)(includes o91 p138)(includes o91 p171)

(waiting o92)
(includes o92 p10)(includes o92 p26)(includes o92 p103)(includes o92 p164)(includes o92 p178)(includes o92 p184)

(waiting o93)
(includes o93 p1)(includes o93 p20)(includes o93 p40)(includes o93 p50)(includes o93 p59)(includes o93 p70)(includes o93 p71)(includes o93 p73)(includes o93 p80)(includes o93 p117)(includes o93 p154)(includes o93 p184)(includes o93 p195)

(waiting o94)
(includes o94 p94)(includes o94 p115)

(waiting o95)
(includes o95 p9)(includes o95 p38)(includes o95 p159)

(waiting o96)
(includes o96 p21)(includes o96 p102)(includes o96 p118)(includes o96 p121)(includes o96 p141)(includes o96 p196)(includes o96 p211)(includes o96 p221)

(waiting o97)
(includes o97 p181)(includes o97 p194)(includes o97 p201)

(waiting o98)
(includes o98 p22)(includes o98 p110)(includes o98 p172)

(waiting o99)
(includes o99 p55)(includes o99 p68)(includes o99 p114)(includes o99 p143)(includes o99 p147)(includes o99 p150)(includes o99 p186)

(waiting o100)
(includes o100 p85)(includes o100 p146)(includes o100 p189)(includes o100 p201)(includes o100 p216)

(waiting o101)
(includes o101 p24)(includes o101 p76)(includes o101 p78)(includes o101 p125)(includes o101 p188)

(waiting o102)
(includes o102 p95)(includes o102 p100)(includes o102 p125)(includes o102 p134)

(waiting o103)
(includes o103 p16)(includes o103 p21)(includes o103 p25)(includes o103 p26)(includes o103 p28)(includes o103 p118)(includes o103 p143)(includes o103 p178)(includes o103 p203)

(waiting o104)
(includes o104 p32)(includes o104 p123)(includes o104 p151)(includes o104 p170)(includes o104 p187)(includes o104 p224)

(waiting o105)
(includes o105 p10)(includes o105 p25)(includes o105 p92)(includes o105 p152)(includes o105 p163)

(waiting o106)
(includes o106 p20)(includes o106 p40)(includes o106 p50)(includes o106 p98)(includes o106 p181)(includes o106 p209)

(waiting o107)
(includes o107 p35)(includes o107 p45)(includes o107 p50)(includes o107 p161)

(waiting o108)
(includes o108 p9)(includes o108 p10)(includes o108 p19)(includes o108 p30)(includes o108 p35)(includes o108 p39)(includes o108 p48)(includes o108 p140)(includes o108 p159)(includes o108 p192)(includes o108 p222)(includes o108 p225)(includes o108 p230)

(waiting o109)
(includes o109 p40)(includes o109 p69)(includes o109 p92)(includes o109 p191)(includes o109 p224)(includes o109 p228)

(waiting o110)
(includes o110 p23)(includes o110 p40)(includes o110 p57)(includes o110 p77)(includes o110 p107)(includes o110 p139)(includes o110 p209)

(waiting o111)
(includes o111 p17)(includes o111 p37)(includes o111 p120)(includes o111 p199)(includes o111 p206)(includes o111 p207)(includes o111 p221)

(waiting o112)
(includes o112 p143)(includes o112 p160)(includes o112 p205)(includes o112 p207)

(waiting o113)
(includes o113 p11)(includes o113 p32)(includes o113 p67)(includes o113 p134)(includes o113 p170)(includes o113 p192)(includes o113 p216)

(waiting o114)
(includes o114 p47)(includes o114 p64)(includes o114 p68)(includes o114 p78)

(waiting o115)
(includes o115 p73)(includes o115 p147)(includes o115 p150)

(waiting o116)
(includes o116 p27)(includes o116 p106)(includes o116 p136)(includes o116 p189)(includes o116 p198)(includes o116 p213)

(waiting o117)
(includes o117 p15)(includes o117 p87)(includes o117 p89)(includes o117 p120)(includes o117 p143)(includes o117 p201)(includes o117 p210)

(waiting o118)
(includes o118 p21)(includes o118 p25)(includes o118 p53)(includes o118 p56)(includes o118 p71)(includes o118 p107)(includes o118 p156)(includes o118 p194)(includes o118 p229)

(waiting o119)
(includes o119 p163)(includes o119 p191)(includes o119 p204)(includes o119 p209)

(waiting o120)
(includes o120 p65)(includes o120 p106)(includes o120 p115)(includes o120 p128)(includes o120 p176)(includes o120 p185)(includes o120 p201)(includes o120 p230)

(waiting o121)
(includes o121 p15)(includes o121 p22)(includes o121 p29)(includes o121 p53)(includes o121 p81)(includes o121 p157)(includes o121 p171)(includes o121 p183)

(waiting o122)
(includes o122 p115)(includes o122 p143)(includes o122 p211)(includes o122 p216)

(waiting o123)
(includes o123 p2)(includes o123 p19)(includes o123 p161)

(waiting o124)
(includes o124 p12)(includes o124 p19)(includes o124 p29)(includes o124 p42)(includes o124 p50)(includes o124 p147)(includes o124 p153)(includes o124 p187)

(waiting o125)
(includes o125 p8)(includes o125 p12)(includes o125 p131)(includes o125 p171)(includes o125 p175)(includes o125 p218)

(waiting o126)
(includes o126 p4)

(waiting o127)
(includes o127 p17)(includes o127 p20)(includes o127 p56)(includes o127 p145)(includes o127 p151)(includes o127 p165)(includes o127 p166)(includes o127 p173)

(waiting o128)
(includes o128 p1)(includes o128 p54)(includes o128 p154)(includes o128 p160)(includes o128 p189)(includes o128 p230)

(waiting o129)
(includes o129 p58)(includes o129 p63)(includes o129 p67)(includes o129 p130)(includes o129 p159)(includes o129 p189)(includes o129 p206)(includes o129 p227)

(waiting o130)
(includes o130 p79)(includes o130 p96)(includes o130 p102)(includes o130 p129)(includes o130 p130)(includes o130 p178)(includes o130 p186)

(waiting o131)
(includes o131 p17)(includes o131 p80)(includes o131 p81)(includes o131 p82)(includes o131 p114)(includes o131 p175)(includes o131 p181)(includes o131 p198)(includes o131 p206)(includes o131 p216)

(waiting o132)
(includes o132 p12)(includes o132 p60)(includes o132 p152)(includes o132 p170)

(waiting o133)
(includes o133 p13)(includes o133 p38)(includes o133 p87)(includes o133 p169)(includes o133 p177)

(waiting o134)
(includes o134 p5)(includes o134 p11)(includes o134 p61)(includes o134 p72)(includes o134 p93)(includes o134 p178)(includes o134 p188)

(waiting o135)
(includes o135 p111)(includes o135 p140)(includes o135 p157)(includes o135 p163)(includes o135 p170)(includes o135 p227)

(waiting o136)
(includes o136 p7)(includes o136 p21)(includes o136 p67)(includes o136 p73)(includes o136 p82)(includes o136 p93)(includes o136 p149)(includes o136 p188)(includes o136 p189)(includes o136 p196)

(waiting o137)
(includes o137 p56)(includes o137 p89)(includes o137 p95)(includes o137 p112)(includes o137 p164)(includes o137 p229)

(waiting o138)
(includes o138 p29)(includes o138 p94)(includes o138 p137)(includes o138 p144)(includes o138 p165)(includes o138 p170)

(waiting o139)
(includes o139 p14)(includes o139 p36)(includes o139 p48)(includes o139 p60)(includes o139 p121)(includes o139 p140)(includes o139 p159)(includes o139 p178)(includes o139 p192)(includes o139 p202)(includes o139 p221)

(waiting o140)
(includes o140 p17)(includes o140 p104)(includes o140 p115)(includes o140 p138)(includes o140 p194)

(waiting o141)
(includes o141 p24)(includes o141 p31)(includes o141 p90)(includes o141 p136)(includes o141 p216)

(waiting o142)
(includes o142 p61)(includes o142 p94)(includes o142 p134)(includes o142 p149)(includes o142 p156)(includes o142 p179)

(waiting o143)
(includes o143 p84)(includes o143 p99)(includes o143 p144)(includes o143 p230)

(waiting o144)
(includes o144 p67)(includes o144 p91)(includes o144 p94)(includes o144 p96)(includes o144 p220)

(waiting o145)
(includes o145 p11)(includes o145 p24)(includes o145 p30)(includes o145 p53)(includes o145 p87)(includes o145 p122)(includes o145 p140)

(waiting o146)
(includes o146 p15)(includes o146 p91)(includes o146 p150)(includes o146 p183)(includes o146 p184)(includes o146 p187)(includes o146 p199)(includes o146 p212)

(waiting o147)
(includes o147 p25)(includes o147 p36)(includes o147 p45)(includes o147 p130)(includes o147 p135)(includes o147 p197)(includes o147 p226)

(waiting o148)
(includes o148 p63)(includes o148 p111)(includes o148 p170)(includes o148 p228)

(waiting o149)
(includes o149 p25)(includes o149 p82)(includes o149 p84)(includes o149 p186)(includes o149 p200)(includes o149 p221)

(waiting o150)
(includes o150 p14)(includes o150 p49)(includes o150 p50)(includes o150 p158)(includes o150 p213)

(waiting o151)
(includes o151 p66)(includes o151 p81)(includes o151 p105)(includes o151 p113)(includes o151 p124)(includes o151 p142)(includes o151 p194)

(waiting o152)
(includes o152 p11)(includes o152 p174)

(waiting o153)
(includes o153 p152)

(waiting o154)
(includes o154 p58)(includes o154 p71)(includes o154 p77)(includes o154 p121)(includes o154 p163)(includes o154 p221)

(waiting o155)
(includes o155 p138)(includes o155 p144)(includes o155 p155)(includes o155 p190)

(waiting o156)
(includes o156 p7)(includes o156 p70)(includes o156 p124)(includes o156 p136)(includes o156 p139)(includes o156 p142)(includes o156 p147)(includes o156 p148)(includes o156 p149)(includes o156 p191)(includes o156 p229)

(waiting o157)
(includes o157 p124)(includes o157 p198)

(waiting o158)
(includes o158 p111)(includes o158 p125)(includes o158 p176)(includes o158 p210)(includes o158 p213)

(waiting o159)
(includes o159 p62)(includes o159 p79)(includes o159 p132)(includes o159 p206)

(waiting o160)
(includes o160 p4)(includes o160 p78)(includes o160 p99)(includes o160 p129)(includes o160 p168)(includes o160 p178)(includes o160 p212)(includes o160 p213)

(waiting o161)
(includes o161 p40)(includes o161 p48)(includes o161 p74)(includes o161 p79)(includes o161 p171)(includes o161 p176)(includes o161 p182)(includes o161 p208)

(waiting o162)
(includes o162 p31)(includes o162 p89)(includes o162 p136)

(waiting o163)
(includes o163 p85)(includes o163 p91)(includes o163 p115)(includes o163 p142)(includes o163 p181)(includes o163 p197)(includes o163 p213)

(waiting o164)
(includes o164 p40)(includes o164 p60)(includes o164 p119)(includes o164 p122)(includes o164 p155)(includes o164 p207)

(waiting o165)
(includes o165 p7)(includes o165 p10)(includes o165 p32)(includes o165 p169)

(waiting o166)
(includes o166 p67)

(waiting o167)
(includes o167 p3)(includes o167 p8)(includes o167 p37)(includes o167 p70)(includes o167 p96)(includes o167 p119)(includes o167 p127)(includes o167 p192)

(waiting o168)
(includes o168 p24)(includes o168 p34)(includes o168 p82)(includes o168 p88)(includes o168 p123)(includes o168 p126)(includes o168 p191)(includes o168 p215)(includes o168 p218)

(waiting o169)
(includes o169 p27)(includes o169 p35)(includes o169 p55)(includes o169 p59)(includes o169 p61)(includes o169 p125)(includes o169 p128)(includes o169 p140)(includes o169 p215)(includes o169 p230)

(waiting o170)
(includes o170 p33)(includes o170 p74)(includes o170 p127)(includes o170 p133)(includes o170 p182)

(waiting o171)
(includes o171 p7)(includes o171 p38)(includes o171 p152)(includes o171 p159)(includes o171 p219)

(waiting o172)
(includes o172 p79)(includes o172 p91)(includes o172 p93)(includes o172 p183)

(waiting o173)
(includes o173 p19)(includes o173 p21)(includes o173 p159)(includes o173 p221)

(waiting o174)
(includes o174 p52)(includes o174 p61)(includes o174 p116)(includes o174 p145)

(waiting o175)
(includes o175 p17)(includes o175 p69)(includes o175 p76)(includes o175 p225)

(waiting o176)
(includes o176 p8)(includes o176 p80)(includes o176 p136)(includes o176 p138)(includes o176 p200)

(waiting o177)
(includes o177 p89)(includes o177 p103)(includes o177 p160)(includes o177 p187)(includes o177 p215)

(waiting o178)
(includes o178 p47)(includes o178 p54)(includes o178 p73)(includes o178 p190)(includes o178 p192)(includes o178 p203)(includes o178 p216)(includes o178 p220)

(waiting o179)
(includes o179 p18)(includes o179 p21)(includes o179 p51)(includes o179 p120)(includes o179 p123)(includes o179 p141)(includes o179 p173)(includes o179 p181)(includes o179 p194)(includes o179 p210)

(waiting o180)
(includes o180 p22)(includes o180 p55)(includes o180 p85)(includes o180 p124)(includes o180 p163)(includes o180 p170)(includes o180 p176)(includes o180 p178)(includes o180 p190)

(waiting o181)
(includes o181 p47)(includes o181 p73)(includes o181 p109)

(waiting o182)
(includes o182 p8)(includes o182 p65)(includes o182 p84)(includes o182 p101)(includes o182 p124)(includes o182 p128)(includes o182 p153)(includes o182 p158)(includes o182 p209)

(waiting o183)
(includes o183 p98)(includes o183 p116)(includes o183 p119)(includes o183 p132)(includes o183 p205)

(waiting o184)
(includes o184 p20)(includes o184 p37)(includes o184 p154)(includes o184 p159)

(waiting o185)
(includes o185 p10)(includes o185 p141)(includes o185 p184)(includes o185 p230)

(waiting o186)
(includes o186 p33)(includes o186 p57)(includes o186 p97)

(waiting o187)
(includes o187 p10)(includes o187 p39)(includes o187 p54)(includes o187 p130)(includes o187 p133)(includes o187 p217)

(waiting o188)
(includes o188 p133)(includes o188 p153)(includes o188 p177)(includes o188 p192)(includes o188 p198)(includes o188 p219)

(waiting o189)
(includes o189 p66)(includes o189 p83)(includes o189 p206)

(waiting o190)
(includes o190 p25)(includes o190 p34)(includes o190 p41)(includes o190 p88)(includes o190 p116)(includes o190 p141)(includes o190 p196)

(waiting o191)
(includes o191 p41)(includes o191 p43)(includes o191 p60)(includes o191 p108)(includes o191 p214)(includes o191 p230)

(waiting o192)
(includes o192 p89)(includes o192 p92)(includes o192 p138)(includes o192 p169)(includes o192 p200)

(waiting o193)
(includes o193 p17)(includes o193 p113)(includes o193 p151)(includes o193 p211)

(waiting o194)
(includes o194 p22)(includes o194 p83)(includes o194 p103)(includes o194 p139)(includes o194 p192)

(waiting o195)
(includes o195 p65)(includes o195 p113)(includes o195 p175)

(waiting o196)
(includes o196 p154)

(waiting o197)
(includes o197 p7)(includes o197 p36)(includes o197 p127)(includes o197 p143)(includes o197 p146)(includes o197 p224)

(waiting o198)
(includes o198 p32)(includes o198 p43)(includes o198 p58)(includes o198 p120)(includes o198 p151)(includes o198 p184)(includes o198 p186)(includes o198 p199)(includes o198 p201)

(waiting o199)
(includes o199 p18)(includes o199 p39)(includes o199 p67)(includes o199 p76)(includes o199 p86)(includes o199 p89)(includes o199 p108)(includes o199 p139)(includes o199 p151)(includes o199 p183)(includes o199 p198)

(waiting o200)
(includes o200 p27)(includes o200 p30)(includes o200 p36)(includes o200 p55)(includes o200 p102)(includes o200 p152)(includes o200 p177)(includes o200 p215)

(waiting o201)
(includes o201 p9)(includes o201 p141)(includes o201 p206)(includes o201 p208)(includes o201 p209)

(waiting o202)
(includes o202 p16)(includes o202 p187)(includes o202 p213)

(waiting o203)
(includes o203 p53)(includes o203 p56)(includes o203 p66)(includes o203 p196)

(waiting o204)
(includes o204 p196)

(waiting o205)
(includes o205 p39)(includes o205 p61)(includes o205 p65)(includes o205 p87)(includes o205 p228)

(waiting o206)
(includes o206 p8)(includes o206 p97)(includes o206 p138)(includes o206 p183)(includes o206 p189)

(waiting o207)
(includes o207 p92)(includes o207 p110)(includes o207 p134)(includes o207 p153)(includes o207 p198)

(waiting o208)
(includes o208 p48)(includes o208 p91)(includes o208 p141)(includes o208 p156)(includes o208 p199)(includes o208 p211)

(waiting o209)
(includes o209 p29)(includes o209 p119)(includes o209 p140)

(waiting o210)
(includes o210 p19)(includes o210 p28)(includes o210 p56)

(waiting o211)
(includes o211 p84)(includes o211 p127)(includes o211 p128)(includes o211 p167)(includes o211 p193)(includes o211 p194)(includes o211 p196)(includes o211 p227)

(waiting o212)
(includes o212 p9)(includes o212 p170)(includes o212 p217)

(waiting o213)
(includes o213 p22)(includes o213 p88)(includes o213 p108)(includes o213 p124)(includes o213 p127)(includes o213 p203)

(waiting o214)
(includes o214 p73)(includes o214 p145)(includes o214 p222)

(waiting o215)
(includes o215 p14)(includes o215 p143)(includes o215 p152)(includes o215 p189)

(waiting o216)
(includes o216 p22)(includes o216 p28)(includes o216 p42)(includes o216 p50)(includes o216 p71)(includes o216 p75)(includes o216 p114)

(waiting o217)
(includes o217 p12)(includes o217 p73)(includes o217 p88)(includes o217 p114)(includes o217 p136)(includes o217 p176)(includes o217 p186)(includes o217 p227)

(waiting o218)
(includes o218 p33)(includes o218 p102)(includes o218 p111)(includes o218 p125)(includes o218 p141)(includes o218 p147)(includes o218 p165)(includes o218 p215)

(waiting o219)
(includes o219 p7)(includes o219 p11)(includes o219 p96)(includes o219 p138)(includes o219 p178)

(waiting o220)
(includes o220 p10)(includes o220 p103)(includes o220 p117)(includes o220 p126)(includes o220 p157)(includes o220 p161)(includes o220 p180)(includes o220 p195)

(waiting o221)
(includes o221 p15)(includes o221 p31)(includes o221 p51)(includes o221 p133)(includes o221 p136)

(waiting o222)
(includes o222 p13)(includes o222 p45)(includes o222 p56)(includes o222 p61)(includes o222 p63)(includes o222 p98)(includes o222 p167)(includes o222 p180)(includes o222 p225)(includes o222 p230)

(waiting o223)
(includes o223 p16)(includes o223 p60)(includes o223 p114)(includes o223 p117)(includes o223 p211)(includes o223 p226)

(waiting o224)
(includes o224 p101)(includes o224 p113)(includes o224 p122)(includes o224 p148)(includes o224 p184)

(waiting o225)
(includes o225 p44)(includes o225 p93)

(waiting o226)
(includes o226 p30)(includes o226 p89)(includes o226 p159)(includes o226 p187)(includes o226 p212)(includes o226 p215)

(waiting o227)
(includes o227 p13)(includes o227 p32)(includes o227 p39)(includes o227 p153)(includes o227 p206)

(waiting o228)
(includes o228 p12)(includes o228 p25)(includes o228 p84)(includes o228 p90)(includes o228 p106)(includes o228 p168)(includes o228 p216)

(waiting o229)
(includes o229 p13)(includes o229 p44)(includes o229 p65)(includes o229 p92)(includes o229 p118)(includes o229 p203)(includes o229 p225)

(waiting o230)
(includes o230 p51)(includes o230 p132)(includes o230 p141)(includes o230 p156)(includes o230 p180)(includes o230 p181)

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
(not-made p191)
(not-made p192)
(not-made p193)
(not-made p194)
(not-made p195)
(not-made p196)
(not-made p197)
(not-made p198)
(not-made p199)
(not-made p200)
(not-made p201)
(not-made p202)
(not-made p203)
(not-made p204)
(not-made p205)
(not-made p206)
(not-made p207)
(not-made p208)
(not-made p209)
(not-made p210)
(not-made p211)
(not-made p212)
(not-made p213)
(not-made p214)
(not-made p215)
(not-made p216)
(not-made p217)
(not-made p218)
(not-made p219)
(not-made p220)
(not-made p221)
(not-made p222)
(not-made p223)
(not-made p224)
(not-made p225)
(not-made p226)
(not-made p227)
(not-made p228)
(not-made p229)
(not-made p230)

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
(shipped o191)
(shipped o192)
(shipped o193)
(shipped o194)
(shipped o195)
(shipped o196)
(shipped o197)
(shipped o198)
(shipped o199)
(shipped o200)
(shipped o201)
(shipped o202)
(shipped o203)
(shipped o204)
(shipped o205)
(shipped o206)
(shipped o207)
(shipped o208)
(shipped o209)
(shipped o210)
(shipped o211)
(shipped o212)
(shipped o213)
(shipped o214)
(shipped o215)
(shipped o216)
(shipped o217)
(shipped o218)
(shipped o219)
(shipped o220)
(shipped o221)
(shipped o222)
(shipped o223)
(shipped o224)
(shipped o225)
(shipped o226)
(shipped o227)
(shipped o228)
(shipped o229)
(shipped o230)
))

(:metric minimize (total-cost))

)


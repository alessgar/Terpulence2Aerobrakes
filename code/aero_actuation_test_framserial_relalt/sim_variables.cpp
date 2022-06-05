#include "state_estimator.h"

float Height[451] = {1.3000000,
1.2800000,
1.3800000,
1.2300000,
1.1900001,
1.1500000,
1.1900001,
1.2500000,
1.2600000,
1.1700000,
1.3700000,
1.3300000,
1.1500000,
1.2600000,
1.3200001,
1.3400000,
1.2500000,
1.2800000,
1.2900000,
1.1799999,
1.3000000,
1.3000000,
1.2600000,
1.1700000,
1.2600000,
1.1700000,
1.1700000,
1.2200000,
1.2300000,
1.2000000,
1.1600000,
1.2200000,
1.2600000,
1.3900000,
1.2500000,
1.1100000,
1.1100000,
1.1500000,
1.1500000,
1.1400000,
1.1900001,
1.2300000,
1.3099999,
1.1600000,
1.1400000,
1.2500000,
1.3400000,
1.3600000,
1.2200000,
1.3000000,
1.2200000,
1.1799999,
1.2000000,
1.0700001,
0.93000001,
1.2700000,
3.5000000,
8.6300001,
16.670000,
27.570000,
41.540001,
53.650002,
65.239998,
76.940002,
88.870003,
101.62000,
115.49000,
129.77000,
144.77000,
161.08000,
178.58000,
196.44000,
215.74001,
235.57001,
254.57001,
274.73001,
295.87000,
314.04001,
336.26001,
366.07001,
396.37000,
424.09000,
450.41000,
475.82999,
499.48999,
523.42999,
547.56000,
571.58002,
594.37000,
616.90997,
639.59003,
661.22998,
682.53003,
704.90997,
726.17999,
747.70001,
768.78003,
789.88000,
811.08002,
832.06000,
853.40997,
873.75000,
894.09998,
914.62000,
934.40002,
954.32001,
973.84998,
993.25000,
1013.9000,
1033.2500,
1052.2000,
1071.1100,
1090.3700,
1109.4301,
1128.2100,
1146.8400,
1165.0300,
1183.3700,
1201.4700,
1219.7900,
1237.6700,
1254.7000,
1272.3700,
1290.0800,
1307.4600,
1324.8900,
1342.2400,
1358.9600,
1375.7500,
1392.5200,
1409.3300,
1425.4399,
1441.3900,
1457.7700,
1473.7900,
1490.0900,
1506.1300,
1522.4100,
1534.0800,
1553.5699,
1566.6700,
1586.6700,
1599.4500,
1619.3300,
1632.4500,
1652.1801,
1664.7900,
1684.3400,
1697.3300,
1716.3600,
1729.2200,
1748.2100,
1760.4301,
1778.9399,
1791.3000,
1809.8300,
1821.5300,
1839.7300,
1851.4700,
1869.1600,
1880.3600,
1897.9600,
1909.2800,
1926.4600,
1937.6000,
1954.1100,
1964.9301,
1981.6899,
1992.0400,
2008.0100,
2018.5000,
2034.6801,
2045.1000,
2060.3799,
2070.5200,
2085.7700,
2095.5400,
2110.5601,
2120.4500,
2135.2400,
2144.5300,
2159.0801,
2168.1599,
2182.5200,
2191.9199,
2205.9399,
2214.8799,
2228.6899,
2237.7900,
2250.9800,
2259.7700,
2272.9800,
2281.5000,
2294.7300,
2302.9800,
2315.9399,
2324.0901,
2336.4299,
2344.5100,
2356.5400,
2364.7400,
2376.6101,
2384.2600,
2395.9299,
2403.4600,
2415.1299,
2422.6699,
2434.1499,
2441.2800,
2452.3501,
2459.4299,
2470.1499,
2477.0200,
2487.9199,
2494.7500,
2505.0701,
2511.9199,
2521.9399,
2528.3601,
2538.5100,
2545.0701,
2555.0300,
2561.1499,
2570.6001,
2576.6499,
2585.6799,
2591.7000,
2600.9600,
2606.8101,
2615.6399,
2621.3000,
2629.7500,
2635.3401,
2643.5701,
2648.8501,
2657.0100,
2662.5000,
2670.7400,
2675.8401,
2683.6799,
2688.6101,
2696.1101,
2700.8301,
2707.9399,
2712.8101,
2719.7100,
2724.3701,
2731.2400,
2735.4099,
2742.3601,
2746.5801,
2753.0300,
2757.3301,
2763.2700,
2767.1799,
2773.1899,
2777.0000,
2783.3601,
2787.1201,
2792.7500,
2796.3201,
2801.8601,
2805.3401,
2810.5300,
2813.8101,
2818.7900,
2822.1299,
2827.0400,
2830.1001,
2834.6499,
2837.6499,
2841.8701,
2844.8101,
2849.2300,
2851.8501,
2855.6899,
2858.3899,
2862.2600,
2864.7700,
2868.2900,
2870.5601,
2873.8999,
2876.2300,
2879.3601,
2881.4900,
2884.3799,
2886.2600,
2889.0601,
2890.8000,
2893.3301,
2895.0601,
2897.4600,
2899.1299,
2901.3301,
2903.2200,
2905.2500,
2907.2000,
2908.7200,
2910.1699,
2911.5601,
2912.8701,
2914.2000,
2915.3301,
2916.3000,
2917.2700,
2918.0300,
2919.1799,
2920.1001,
2920.9199,
2921.5300,
2922.0400,
2922.4199,
2922.8101,
2923.2800,
2923.7600,
2921.1001,
2917.6299,
2921.5901,
2923.1399,
2923.8601,
2923.8999,
2923.8601,
2923.6299,
2923.1399,
2922.5000,
2921.5500,
2920.8101,
2919.9800,
2919.3201,
2918.5701,
2917.6499,
2916.8501,
2916.2200,
2914.7000,
2912.8799,
2910.2600,
2907.7100,
2904.8899,
2901.8000,
2899.3799,
2897.2700,
2895.8601,
2893.0901,
2891.5100,
2889.1599,
2888.1201,
2887.2400,
2887.9600,
2889.4700,
2891.1499,
2890.3799,
2888.5400,
2884.2800,
2881.2600,
2876.8201,
2874.4800,
2870.5701,
2867.2200,
2861.0000,
2857.6799,
2853.6399,
2851.3501,
2848.0300,
2845.7200,
2842.9900,
2841.7300,
2839.8799,
2838.8701,
2837.4199,
2838.1499,
2836.9800,
2836.8101,
2838.0500,
2837.2100,
2831.8000,
2826.0000,
2815.9099,
2809.8999,
2803.7100,
2801.3301,
2797.9399,
2795.7800,
2792.0801,
2789.2400,
2785.3000,
2782.3401,
2778.1799,
2775.9900,
2774.0100,
2773.2200,
2770.9399,
2629.2100,
2386.0500,
2307.1101,
2245.3899,
2223.8501,
2206.1201,
2199.4800,
2194.3000,
2192.3301,
2184.2300,
2180.7100,
2174.2300,
2169.7100,
2161.5400,
2155.7400,
2147.6201,
2143.0100,
2136.3401,
2131.8501,
2125.1499,
2121.1799,
2115.1399,
2111.5400,
2105.9399,
2102.8301,
2097.9199,
2094.0200,
2085.2700,
2080.3799,
2073.4500,
2069.5000,
2064.8899,
2062.5500,
2057.4500,
2052.6599,
2043.9200,
2039.1801,
2032.6500,
2028.6300,
2023.4000,
2020.3900,
2014.6899,
2011.2200,
2005.1100,
2000.3800,
1993.4900,
1989.8400,
1986.1400,
1983.7900,
1978.6899,
1974.4500,
1966.1600,
1961.5699,
1955.7400,
1951.8400,
1944.2800,
1939.4000,
1929.7400,
1923.8500,
1915.1000,
};

float Accel[451] = {-9.7600002,
-9.7700005,
-9.7600002,
-9.7600002,
-9.7700005,
-9.7600002,
-9.7500000,
-9.7700005,
-9.7600002,
-9.7500000,
-9.7700005,
-9.7600002,
-9.7700005,
-9.7700005,
-9.7600002,
-9.7700005,
-9.7700005,
-9.7600002,
-9.7500000,
-9.7500000,
-9.7600002,
-9.7600002,
-9.7700005,
-9.7600002,
-9.7600002,
-9.7700005,
-9.7600002,
-9.7600002,
-9.7700005,
-9.7600002,
-9.7700005,
-9.7700005,
-9.7700005,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7500000,
-9.7600002,
-9.7500000,
-9.7600002,
-9.7600002,
-9.7799997,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7600002,
-9.7500000,
-9.7600002,
-9.7500000,
-9.7600002,
-10.420000,
-117.85000,
-117.31000,
-120.22000,
-122.63000,
-124.55000,
-125.94000,
-126.35000,
-126.61000,
-126.81000,
-126.66000,
-125.98000,
-124.91000,
-123.66000,
-122.01000,
-120.23000,
-118.24000,
-115.77000,
-113.06000,
-110.01000,
-107.66000,
-105.78000,
-96.220001,
-66.879997,
-19.709999,
7.4800000,
14.280000,
15.840000,
16.870001,
17.139999,
16.900000,
16.709999,
16.309999,
16.420000,
16.580000,
15.950000,
15.720000,
15.130000,
15.480000,
14.680000,
14.650000,
14.880000,
14.340000,
13.860000,
13.550000,
13.220000,
12.920000,
12.710000,
12.610000,
12.460000,
12.310000,
12.190000,
11.850000,
11.680000,
11.460000,
11.230000,
10.950000,
11.120000,
10.820000,
10.480000,
10.260000,
10.040000,
9.9200001,
9.7200003,
9.3900003,
9.3500004,
9.2500000,
9.3000002,
8.8699999,
8.8000002,
8.5200005,
8.4600000,
8.2200003,
8.1899996,
7.9000001,
7.8400002,
7.7199998,
7.6500001,
7.6999998,
7.4099998,
7.2399998,
7.1300001,
7.1100001,
6.8200002,
6.7399998,
6.6999998,
6.4499998,
6.4600000,
6.3499999,
6.2199998,
6.1999998,
6.0100002,
5.8800001,
5.8200002,
5.7399998,
5.5500002,
5.5100002,
5.4499998,
5.2600002,
5.1999998,
5.1799998,
5.1700001,
4.8800001,
4.7399998,
4.8099999,
4.5700002,
4.6199999,
4.5599999,
4.4499998,
4.3299999,
4.2800002,
4.1199999,
4.0400000,
3.9400001,
3.9700000,
3.9600000,
3.9300001,
3.7900000,
3.7000000,
3.5799999,
3.5400000,
3.4900000,
3.5100000,
3.3499999,
3.3000000,
3.1700001,
3.1800001,
3.1099999,
3.0899999,
3.0599999,
3.0100000,
2.8900001,
2.8499999,
2.7900000,
2.7300000,
2.7100000,
2.6700001,
2.6099999,
2.5300000,
2.5400000,
2.4300001,
2.3900001,
2.3299999,
2.3000000,
2.2500000,
2.2300000,
2.2100000,
2.2100000,
2.0799999,
2.1099999,
2.0400000,
2.0000000,
1.9299999,
1.9100000,
1.8600000,
1.8000000,
1.7600000,
1.7500000,
1.7500000,
1.7000000,
1.6500000,
1.6500000,
1.5900000,
1.5300000,
1.4900000,
1.4600000,
1.4600000,
1.4000000,
1.3500000,
1.3300000,
1.3300000,
1.2700000,
1.2600000,
1.2500000,
1.2200000,
1.1900001,
1.1700000,
1.1200000,
1.1500000,
1.0800000,
1.0599999,
1.0200000,
1.0200000,
0.99000001,
0.94999999,
0.94000000,
0.89999998,
0.85000002,
0.83999997,
0.82999998,
0.82999998,
0.79000002,
0.79000002,
0.74000001,
0.74000001,
0.69999999,
0.69999999,
0.68000001,
0.66000003,
0.63999999,
0.61000001,
0.60000002,
0.58999997,
0.56999999,
0.56000000,
0.54000002,
0.54000002,
0.50000000,
0.49000001,
0.47000000,
0.47000000,
0.44000000,
0.41000000,
0.41000000,
0.40000001,
0.38000000,
0.34000000,
0.34000000,
0.33000001,
0.33000001,
0.30000001,
0.28999999,
0.28000000,
0.25999999,
0.25999999,
0.25999999,
0.23999999,
0.22000000,
0.23000000,
0.22000000,
0.22000000,
0.20000000,
0.18000001,
0.19000000,
0.18000001,
0.16000000,
0.18000001,
0.18000001,
0.17000000,
0.16000000,
0.16000000,
0.15000001,
0.14000000,
0.14000000,
0.13000000,
0.11000000,
0.10000000,
0.11000000,
0.090000004,
0.079999998,
0.070000000,
0.059999999,
0.050000001,
0.050000001,
0.020000000,
0.020000000,
0.020000000,
0.020000000,
0.0099999998,
0.0099999998,
-0.020000000,
59.169998,
1.3400000,
0.079999998,
-0.05000001,
0.050000001,
0.050000001,
0.059999999,
-0.020000000,
0.039999999,
-0.079999998,
-0.050000001,
-0.020000000,
-0.039999999,
-0.039999999,
-0.020000000,
-0.51999998,
-0.52999997,
-0.14000000,
-0.12000000,
-1.7000000,
-1.0900000,
-0.18000001,
-0.17000000,
-0.52999997,
-0.87000000,
-2.3599999,
-2.3000000,
-2.3399999,
-3.0100000,
-4.6900001,
-4.9899998,
-5.2399998,
-6.3099999,
-9.5400000,
-10.830000,
-5.6799998,
-3.8699999,
-3.7500000,
-3.7000000,
-2.1600001,
-2.5200000,
-2.2300000,
-1.9800000,
-3.4200001,
-3.2900000,
-2.4600000,
-1.7300000,
-1.5800000,
-1.7700000,
-1.9000000,
-1.8600000,
-1.7000000,
-1.1700000,
-0.69000000,
-0.63999999,
-1.1300000,
-2.7300000,
-3.5100000,
-4.3499999,
-4.3000002,
-5.6199999,
-6.6700001,
-5.7700000,
-6.3200002,
-7.5100002,
-5.7100000,
-3.8199999,
-4.6999998,
-6.3200002,
-9.3900003,
-8.1999998,
-6.8699999,
-7.8299999,
-7.3099999,
-7.4800000,
-7.4800000,
-8.8100004,
-9.5299997,
-9.5900002,
-5.3299999,
-8.9600000,
-7.6300001,
-6.1599998,
-4.4099998,
-8.1099997,
-1.5900000,
-3.1800001,
-3.3699999,
-5.3400002,
-3.8800001,
-5.8699999,
-10.900000,
-8.5500002,
-3.6199999,
-5.2600002,
-7.4699998,
-10.990000,
-9.3000002,
-7.7399998,
-3.5000000,
-6.7700000,
-8.7100000,
-8.3699999,
-4.3499999,
-8.2799997,
-5.2199998,
-5.3699999,
-8.0200005,
-8.3000002,
-10.270000,
-7.4299998,
-8.9099998,
-7.1700001,
-8.5699997,
-10.750000,
-7.0799999,
-9.1400003,
-7.6399999,
-4.5500002,
-8.6899996,
-7.1900001,
-7.4499998,
-3.4800000,
-7.0900002,
-5.5900002,
-3.5100000,
-3.7300000,
-2.0999999,
-2.5000000,
-3.8000000,
-5.1599998,
-6.8200002,
-7.1300001,
-8.4499998,
-10.220000,
-11.470000};


float Time[451]= {0,
0.16000000,
0.31999999,
0.47999999,
0.63000000,
0.85000002,
1.0100000,
1.1700000,
1.3300000,
1.4900000,
1.7100000,
1.8700000,
2.0300000,
2.1900001,
2.4100001,
2.9600000,
3.1199999,
3.2800000,
3.4500000,
3.6700001,
3.8299999,
4.0000000,
4.1599998,
4.3299999,
4.5599999,
4.7199998,
4.8899999,
5.0599999,
5.2199998,
5.4499998,
5.6199999,
5.7900000,
5.9600000,
6.1199999,
6.3600001,
6.5200000,
6.6999998,
6.8600001,
7.0300002,
7.2700000,
7.4400001,
7.6100001,
7.7800002,
7.9600000,
8.1999998,
8.3699999,
8.5400000,
8.7100000,
8.8900003,
9.1300001,
9.3000002,
9.4799995,
9.6499996,
9.8299999,
10.070000,
10.250000,
10.420000,
10.600000,
10.840000,
11.020000,
11.200000,
11.380000,
11.480000,
11.570000,
11.660000,
11.760000,
11.850000,
11.950000,
12.040000,
12.130000,
12.230000,
12.320000,
12.410000,
12.510000,
12.600000,
12.700000,
12.790000,
12.880000,
12.980000,
13.070000,
13.150000,
13.240000,
13.320000,
13.400000,
13.480000,
13.570000,
13.650000,
13.730000,
13.820000,
13.900000,
13.980000,
14.070000,
14.150000,
14.230000,
14.310000,
14.400000,
14.480000,
14.560000,
14.650000,
14.730000,
14.810000,
14.890000,
14.980000,
15.060000,
15.140000,
15.230000,
15.310000,
15.390000,
15.480000,
15.560000,
15.640000,
15.730000,
15.810000,
15.890000,
15.980000,
16.059999,
16.139999,
16.219999,
16.309999,
16.389999,
16.469999,
16.559999,
16.639999,
16.719999,
16.809999,
16.889999,
16.980000,
17.059999,
17.139999,
17.230000,
17.309999,
17.389999,
17.480000,
17.559999,
17.639999,
17.730000,
17.809999,
17.900000,
18.000000,
18.080000,
18.190001,
18.260000,
18.370001,
18.450001,
18.549999,
18.629999,
18.740000,
18.809999,
18.920000,
19.000000,
19.100000,
19.180000,
19.290001,
19.360001,
19.469999,
19.549999,
19.650000,
19.730000,
19.840000,
19.910000,
20.020000,
20.100000,
20.200001,
20.280001,
20.389999,
20.469999,
20.570000,
20.650000,
20.750000,
20.830000,
20.940001,
21.020000,
21.120001,
21.200001,
21.309999,
21.379999,
21.490000,
21.570000,
21.670000,
21.750000,
21.860001,
21.930000,
22.040001,
22.120001,
22.219999,
22.299999,
22.410000,
22.490000,
22.590000,
22.670000,
22.770000,
22.850000,
22.959999,
23.040001,
23.139999,
23.219999,
23.330000,
23.400000,
23.510000,
23.590000,
23.690001,
23.770000,
23.870001,
23.950001,
24.059999,
24.139999,
24.240000,
24.320000,
24.420000,
24.500000,
24.610001,
24.690001,
24.790001,
24.870001,
24.980000,
25.049999,
25.160000,
25.240000,
25.340000,
25.420000,
25.530001,
25.600000,
25.709999,
25.790001,
25.889999,
25.969999,
26.080000,
26.150000,
26.260000,
26.340000,
26.440001,
26.520000,
26.620001,
26.700001,
26.809999,
26.889999,
26.990000,
27.070000,
27.180000,
27.250000,
27.360001,
27.440001,
27.540001,
27.620001,
27.730000,
27.799999,
27.910000,
27.990000,
28.090000,
28.170000,
28.270000,
28.350000,
28.459999,
28.540001,
28.639999,
28.719999,
28.830000,
28.900000,
29.010000,
29.090000,
29.190001,
29.270000,
29.370001,
29.450001,
29.559999,
29.639999,
29.740000,
29.820000,
29.920000,
30.000000,
30.110001,
30.190001,
30.290001,
30.370001,
30.469999,
30.549999,
30.660000,
30.740000,
30.840000,
30.920000,
31.020000,
31.100000,
31.209999,
31.280001,
31.389999,
31.469999,
31.570000,
31.650000,
31.750000,
31.830000,
31.940001,
32.020000,
32.119999,
32.200001,
32.290001,
32.389999,
32.480000,
32.580002,
32.669998,
32.759998,
32.860001,
32.950001,
33.040001,
33.139999,
33.230000,
33.330002,
33.419998,
33.509998,
33.610001,
33.700001,
33.799999,
33.889999,
33.990002,
34.080002,
34.169998,
34.270000,
34.360001,
34.459999,
34.560001,
34.650002,
34.740002,
34.840000,
34.930000,
35.029999,
35.119999,
35.209999,
35.310001,
35.400002,
35.500000,
35.590000,
35.680000,
35.779999,
35.869999,
35.970001,
36.060001,
36.150002,
36.250000,
36.340000,
36.439999,
36.529999,
36.619999,
36.730000,
36.810001,
36.910000,
36.990002,
37.099998,
37.180000,
37.279999,
37.360001,
37.459999,
37.540001,
37.650002,
37.730000,
37.830002,
37.910000,
38.009998,
38.090000,
38.200001,
38.270000,
38.380001,
38.459999,
38.560001,
38.639999,
38.750000,
38.820000,
38.930000,
39.009998,
39.110001,
39.189999,
39.299999,
39.369999,
39.480000,
39.560001,
39.660000,
39.740002,
39.849998,
39.930000,
40.029999,
40.110001,
40.220001,
40.290001,
40.400002,
40.480000,
40.580002,
40.660000,
40.770000,
40.840000,
40.950001,
41.029999,
41.130001,
41.209999,
57.560001,
57.630001,
57.790001,
57.950001,
58.110001,
58.240002,
58.400002,
58.560001,
58.720001,
58.860001,
59.060001,
59.189999,
59.349998,
59.470001,
59.660000,
59.790001,
59.950001,
60.080002,
60.240002,
60.400002,
60.560001,
60.689999,
60.849998,
61.020000,
61.180000,
61.310001,
61.470001,
61.610001,
61.810001,
61.939999,
62.099998,
62.240002,
62.439999,
62.570000,
62.740002,
62.869999,
63.029999,
63.209999,
63.380001,
63.509998,
63.680000,
63.820000,
63.980000,
64.120003,
64.290001,
64.419998,
64.639999,
64.779999,
64.949997,
65.089996,
65.260002,
65.449997,
65.610001,
65.760002,
65.930000,
66.120003,
66.290001,
66.430000,
66.599998,
66.750000};

int i=1;
int j=0;
float sim_height=0;
float sim_accel=0;

float sim_Height(float current_time){
// make sure you first run the sim_height before the sim_accel
  while (i){
    if (j<453){    
      if (current_time>=Time[j]){
      sim_height=Height[j];
      i=0;}
      else{
      j++;
      } 
    }
    else{
      sim_height=0;
      i=0;
    }
 } 
  i=1;
return sim_height;
}

float sim_Accel(){
sim_accel=Accel[j];
return sim_accel;
}

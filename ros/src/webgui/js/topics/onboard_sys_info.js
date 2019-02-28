var onboard_sys_info = new ROSLIB.Topic({
    ros : ros,
    name : '/monitor/msg',
    messageType : 'jetson_data_msg'
  });

onboard_sys_info.subscribe(function(message) {
    //console.log('Received message on ' + depth_node.name + ': ' + message.temperature); 
    $('#ram_usage').text(RAM_N + "/" + RAM_D);
    $('#cpu_1').text(CPU_usage_1);
    $('#cpu_2').text(CPU_usage_2);
    $('#cpu_3').text(CPU_usage_3);
    $('#cpu_4').text(CPU_usage_4);
    $('#cpu_5').text(CPU_usage_5);
    $('#cpu_6').text(CPU_usage_6);
    $('#bcpu_temp').text(BCPU_temp);
    $('#mcpu_temp').text(MCPU_temp);
    $('#gpu_temp').text(GPU_temp);
    $('#pll_temp').text(PLL_temp);
    $('#tboard_temp').text(Tboard_temp);
    $('#tdiode_temp').text(Tdiode_temp);
    $('#pmic_temp').text(PMIC_temp);
    $('#thermal').text(thermal);
    $('#vdd_in').text(VDD_IN_N + "/" + VDD_IN_D);
    $('#vdd_cpu').text(VDD_CPU_N + "/" + VDD_CPU_D);
    $('#vdd_gpu').text(VDD_GPU_N + "/" + VDD_GPU_D);
    $('#vdd_soc').text(VDD_SOC_N + "/" + VDD_SOC_D);
    $('#vdd_wifi').text(VDD_WIFI_N + "/" + VDD_WIFI_D);
    $('#vdd_ddr').text(VDD_DDR_N + "/" + VDD_DDR_D);
  });
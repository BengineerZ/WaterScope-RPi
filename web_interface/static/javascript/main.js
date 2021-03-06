Vue.options.delimiters = ["[[", "]]"];

var app = new Vue({
    el: '#app',
    data: () => ({
        serial_command: '',
        peltier_control_command: '',
        chosen_arduino_board: "waterscope",
        available_arduino_boards: ['waterscope'],
        LED_switch: 'true',
        UV_switch: null,
        defogger_switch: 'false',
        stream_method: 'PiCamera',
        recording_switch: null,
        config_update_switch: null,
        timelapse_switch: null,
        timelapse_interval: 10,
        zoom: 1,
        max_zoom: 5,
        temp_plot: true,
        alert_window: false,
        alert_window_timeout: 5000,
        alert_window_2: false,
        alert_window_2_timeout: 1000,
        alert_content_2: '',
        default_LED_RGB: (30,30,30),
    }),

    // watch when data change 
    watch: {
        zoom: function () {
            this.change_zoom()
        },
        LED_switch: function () {
            if (this.LED_switch == "true") {
                this.LED_on()
            } else if (this.LED_switch == null) {
                this.LED_off()
            }
            this.alert_window = true
        },
        UV_switch: function () {
            if (this.UV_switch == "true") {
                this.UV_on()
            } else if (this.UV_switch == null) {
                this.UV_off()
            }
            this.alert_window = true
        },
        defogger_switch: function () {
            if (this.defogger_switch == "true") {
                this.defogger_on()
            } else if (this.defogger_switch == null) {
                this.defogger_off()
            }
            this.alert_window = true
        },
        recording_switch: function () {
            if (this.recording_switch == "true") {
                this.record_video(recording = true, filename = 'rasbperry_pi_time')
            } else if (this.recording_switch == null) {
                this.record_video(recording = false)
            }
            this.alert_window = true
        },
        config_update_switch: function () {
            if (this.config_update_switch == "true") {
                this.start_config_update()
            } else if (this.config_update_switch == null) {
                this.stop_config_update()
            }
            this.alert_window = true
        },
        timelapse_switch: function () {
            // always stop the timelapse first
            this.stop_timelapse()
            if (this.timelapse_switch == 'timelapse') {
                this.start_timelapse()
            } else if (this.timelapse_switch == 'waterscope_timelapse') {
                this.start_waterscope_timelapse()
            } else if (this.timelapse_switch == null || this.timelapse_switch == 'stop_timelapse') {
                this.stop_timelapse()
            }

            this.alert_window = true
        },
    },

    computed: {
        alert_content: function () {
            alert_content = []
            if (this.LED_switch != null) {
                alert_content.push("LED")
            }
            if (this.UV_switch != null) {
                alert_content.push("UV")
            }
            if (this.defogger_switch != null) {
                alert_content.push("defogger")
            }
            if (this.config_update_switch != null) {
                alert_content.push('Updating config')
            }
            if (this.recording_switch != null) {
                alert_content.push('Recording')
            }
            if (this.timelapse_switch != null) {
                alert_content.push('Timelapse')
            }
            return alert_content.join(', ')
        },
    },

    mounted: function () {
        this.read_server_info()
        this.LED_switch = "true"
        this.defogger_switch = "true"
        setTimeout(() => {
            this.serial_command = 'LED_RGB={0}'.format(this.default_LED_RGB);
            this.send_serial_command();
            this.set_pi_time_with_user_time();
        }, 2000)
        
        setInterval(() => {
            this.read_auto_focus_status();
        }, 200)

        setInterval(() => {
            this.handle_income_serial_command();
        }, 1000)
    },

    methods: {
        LED_on: function () {
            console.log('turn on LED')
            axios.get("/send_serial/?value=LED_on&board={0}".format(this.chosen_arduino_board));
        },
        LED_off: function () {
            console.log('turn off LED')
            axios.get("/send_serial/?value=LED_off&board={0}".format(this.chosen_arduino_board))
        },
        UV_on: function () {
            console.log('turn on UV')
            axios.get("/send_serial/?value=UV_on&board={0}".format(this.chosen_arduino_board))
        },
        UV_off: function () {
            console.log('turn off UV')
            axios.get("/send_serial/?value=UV_off&board={0}".format(this.chosen_arduino_board))
        },
        defogger_on: function () {
            console.log('turn on defogger')
            axios.get("/send_serial/?value=defogger_on&board={0}".format(this.chosen_arduino_board));
        },
        defogger_off: function () {
            console.log('turn off defogger')
            axios.get("/send_serial/?value=defogger_off&board={0}".format(this.chosen_arduino_board))
        },
        start_config_update: function () {
            console.log('start updating config')
            this.config_loop = setInterval(() => {
                axios.get("/settings/?config_update=true");
            }, 200)
        },
        stop_config_update: function () {
            console.log('stop updating config')
            clearInterval(this.config_loop)
        },

        start_timelapse: function () {
            if (this.timelapse_interval < 1.5) {
                this.timelapse_interval = 1.5
            }
            axios.get('/acquire_data/?option=high_res_timelapse_{0}&filename=raspberry_pi_time'.format(this.timelapse_interval))
            // axios.get('/acquire_data/?option=normal_timelapse_{0}&filename=raspberry_pi_time'.format(this.timelapse_interval))            
        },
        start_waterscope_timelapse: function () {
            if (this.timelapse_interval < 10) {
                this.timelapse_interval = 10
            }
            axios.get('/acquire_data/?option=waterscope_timelapse_{0}&filename=raspberry_pi_time'.format(this.timelapse_interval))
        },
        stop_timelapse: function () {
            axios.get('/acquire_data/?option=stop_timelapse')
        },

        change_stream_method: function () {
            // have a slight delay for the stream_method to change
            setTimeout(() => {
                if (this.stream_method == 'PiCamera' || this.stream_method == null) {
                    console.log('changing streaming: {0}'.format(this.stream_method))
                    axios.get('/change_stream_method/?stream_method=PiCamera')
                    console.log('change to picamera')
                    // TODO: maybe we can refresh the jpg elements
                    this.refresh()

                } else if (this.stream_method == 'OpenCV') {
                    console.log('changing streaming: {0}'.format(this.stream_method))
                    axios.get('/change_stream_method/?stream_method=OpenCV')
                    console.log('change to opencv')
                    this.refresh()
                }
            }, 100)
            this.alert_window = true
        },

        send_serial_command: function () {
            console.log('Sending serial command {0}'.format(this.serial_command))
            axios.get('/send_serial/?value={0}&board={1}'.format(this.serial_command, this.chosen_arduino_board))
            // alert('Sending serial command \n {0}'.format(this.serial_command))
            // remove focus of the text field
            app.$refs.serial_command_field.blur()
            // show the pop up alert
            this.alert_window = true
            // a special function to record the current offset temperature and update other T0, T1..
            this.measure_offset_temp()
            // scroll to the bottom automatically
            // window.scrollTo(0,2000);
        },

        // a special function to record the current offset temperature
        measure_offset_temp: function () {
            if (this.serial_command.includes('offset=') == true) {
                offset_temp = parseFloat(this.serial_command.substring(7))
                this.Tprep = 21 + offset_temp
                this.Theat = 25 + offset_temp
                this.T0 = 20 + offset_temp
                this.T1 = 19 + offset_temp
                this.T2 = 18 + offset_temp
                this.T3 = 17 + offset_temp
            }
        },

        read_server_info: function () {
            // Flask sending all the information through this page
            axios
                .get('/settings/')
                .then(response => {
                    this.stream_method = response.data.stream_method;
                    this.available_arduino_boards = response.data.available_arduino_boards;
                    this.default_LED_RGB = response.data.default_LED_RGB;
                })
        },
        read_auto_focus_status: function(){
            axios.get('/auto_focus/?command=auto_focus_status').then(response => {
                // disable the keyboard control of the motor when it is busy
                auto_focus_status = response.data.auto_focus_status
                console.log(auto_focus_status)
            })
            if (auto_focus_status == 'done'){
                axios.get('/auto_focus/?command=reset')
                // axios.get("/send_serial/?value=LED_RGB=0,50,0&board={0}".format(this.chosen_arduino_board))

                setTimeout(() => {
                // axios.get("/send_serial/?value=LED_RGB=10,10,10&board={0}".format(this.chosen_arduino_board))
                axios.get("/send_serial/?value=af_complete&board={0}".format(this.chosen_arduino_board))
                
            }, 500);

                // this.refresh()
            }
        },
        handle_income_serial_command: function(){
            var vm = this
            axios.get('/income_serial_command/?command=read').then(response => {
                var income_serial_command  = response.data.income_serial_command
                // console.log(response)
                // clear the cached command after receiving it
                if (income_serial_command == 'auto_focus'){
                    console.log('auto_focus')
                    axios.get('/income_serial_command/?command=clear')
                    vm.auto_focus()
                }
                if (income_serial_command == 'capture'){
                    axios.get('/income_serial_command/?command=clear')
                    console.log('take photo')
                    vm.take_image("high_res", "raspberry_pi_time")
                }
            })
        },
        set_pi_time_with_user_time: function () {
            var user_time = new Date();
            var user_time_formatted = user_time.getUTCFullYear() + '-' + (user_time.getUTCMonth()+1) + '-' + user_time.getUTCDate() + ' ' + user_time.getUTCHours() + ":" + user_time.getUTCMinutes() + ":" + user_time.getUTCSeconds() + " UTC"
            axios.get('/update_time/?user_time={0}'.format(user_time_formatted))
            console.log('updated system time with: ' + user_time_formatted)
        },

        change_zoom: function () {
            axios.get('/settings/?zoom_value={0}'.format(this.zoom))
        },
        zoom_in: function () {
            console.log('zoom in');
            this.zoom = (this.zoom + 0.5) || this.max_zoom
        },
        zoom_out: function () {
            console.log('zoom out');
            this.zoom = (this.zoom - 0.5) || 1
        },
        refresh: function () {
            window.location = "/"
        },
        toggle_temp_plot: function () {
            this.temp_plot = !this.temp_plot
        },
        stop_stream: function () {
            axios.get("/settings/?stop=true");
            this.stop_config_update()
            this.stop_timelapse()
            this.record_video(recording = false)
        },
        auto_focus: function () {
            console.log('starting the auto focus now')
            // start the auto focusing
            // axios.get("/send_serial/?value=LED_RGB=50,0,0&board={0}".format(this.chosen_arduino_board))

            // setTimeout(() => {
            //     axios.get("/send_serial/?value=LED_RGB=10,10,10&board={0}".format(this.chosen_arduino_board))
            // }, 500);

            axios.get('/auto_focus/?command=start')
            // setTimeout(() => {
            //     this.refresh()
            // }, 1000);
            // window.location.href = '/auto_focus/?command=start'
            //  NOTE: have some way to go back to PiCamera stream after auto focusing?
        },
        take_image: function (option = '', filename = 'raspberry_pi_time') {
            axios.get("/acquire_data/?option={0}&filename={1}".format(option, filename))
            console.log("taking image: option: {0}, time: {1}".format(option, filename))
            this.alert_window_2 = true
            this.alert_content_2 = 'taking image..'
            if (this.recording_switch != null) {
                this.alert_content_2 = 'recording video, taking image'
                setTimeout(() => {
                    this.alert_content_2 = 'recording video...'
                }, 1000);
            }
        },
        record_video: function (recording = true, filename = 'raspberry_pi_time') {
            if (recording == true) {
                // Take an image before recording to note the time
                this.take_image(option = '', filename = 'raspberry_pi_time_start_recording')
                axios.get("/acquire_data/?option=start_recording_video&filename={0}".format(filename))
                console.log("recording video")
                this.alert_window_2 = true
                this.alert_content_2 = 'recording video...'
                this.alert_window_2_timeout = 1000 * 500
            } else {
                // Take an image after recording to note the time
                this.take_image(option = '', filename = 'raspberry_pi_time_stop_recording')
                console.log('stop recording')
                axios.get("/acquire_data/?option=stop_recording_video")
                this.photo_capture_status = ''
                this.alert_window_2 = false
                this.alert_window_2_timeout = 1000
            }
        },
    }
})

// NOTE: python-like string format method https://coderwall.com/p/flonoa/simple-string-format-in-javascript
String.prototype.format = function () {
    var str = this;
    for (var i in arguments) {
        str = str.replace(new RegExp("\\{" + i + "\\}", 'g'), arguments[i]);
    }
    return str
}
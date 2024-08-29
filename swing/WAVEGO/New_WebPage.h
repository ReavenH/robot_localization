static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>

<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32 CAM Robot</title>
    <style>
    body {
        font-family: Arial, Helvetica, sans-serif;
        background: #000000;
        color: #efefef;
        font-size: 16px
    }

    h2 {
        font-size: 18px
    }

    section.main {
        display: flex
    }

    #content {
        display: flex;
        flex-wrap: wrap;
        align-items: stretch
    }

    figure {
        padding: 0;
        margin: 0;
        -webkit-margin-before: 0;
        margin-block-start: 0;
        -webkit-margin-after: 0;
        margin-block-end: 0;
        -webkit-margin-start: 0;
        margin-inline-start: 0;
        -webkit-margin-end: 0;
        margin-inline-end: 0
    }

    figure img {
        display: block;
        width: 100%;
        height: auto;
        border-radius: 4px;
        margin-top: 8px
    }

    @media (min-width:800px) and (orientation:landscape) {
        #content {
            display: flex;
            flex-wrap: nowrap;
            align-items: stretch
        }

        figure img {
            display: block;
            max-width: 100%;
            max-height: calc(100vh - 40px);
            width: auto;
            height: auto
        }

        figure {
            padding: 0;
            margin: 0;
            -webkit-margin-before: 0;
            margin-block-start: 0;
            -webkit-margin-after: 0;
            margin-block-end: 0;
            -webkit-margin-start: 0;
            margin-inline-start: 0;
            -webkit-margin-end: 0;
            margin-inline-end: 0
        }
    }

    section #buttons {
        display: flex;
        flex-wrap: nowrap;
        justify-content: space-between
    }

    button {
        display: block;
        margin: 5px;
        padding: 5px 12px;
        border: 0;
        line-height: 28px;
        cursor: pointer;
        color: #fff;
        background: #4247b7;
        border-radius: 5px;
        font-size: 16px;
        outline: 0;
        width: 100px

        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;

        user-select: none;
    }

    .button2 {
        background-color: #1cb8bd;
        width: 100px
    }

    .button4 {
        background-color: #e7e7e7;
        color: #000;
        width: 120px
    }

    button:hover {
        background: #ff494d
    }

    button:active {
        background: #f21c21
    }

    .image-container {
        position: absolute;
        top: 50px;
        left: 50%;
        margin-right: -50%;
        transform: translate(-50%, -50%);
        min-width: 160px
    }

    .control-container {
        position: relative;
        top: 400px;
        left: 50%;
        margin-right: -50%;
        transform: translate(-50%, -50%)
    }

    .close {
        position: absolute;
        right: 5px;
        top: 5px;
        background: #ff3034;
        width: 16px;
        height: 16px;
        border-radius: 100px;
        color: #fff;
        text-align: center;
        line-height: 18px;
        cursor: pointer
    }

    .hidden {
        display: none
    }

    .rotate0 {
        -webkit-transform: rotate(0deg);
        -moz-transform: rotate(0deg);
        -o-transform: rotate(0deg);
        -ms-transform: rotate(0deg);
        transform: rotate(0deg)
    }
    </style>
</head>

<body>
    <br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br><br>
    <section class="main">
        <figure>
            <div id="stream-container" class="image-container">
                <br><br><br><br><br><br><br><br><br>
                <div class="close" id="close-stream">×</div>
                <img id="stream" src="" class="rotate0">
            </div>
        </figure>
        <section id="buttons">
            <div id="controls" class="control-container">
                <table>
                    <tr>
                        <td></td>
                        <td align="center"><button class="button" id="toggle-stream">Start</button></td>
                        <td></td>
                    </tr>
                    <tr>
                    <br><br><br>
                    </tr>
                    <tr>
                        <td></td>
                        <td align="center"><button class="button" id="forward" onmousedown="fetch(document.location.origin+'/control?var=move&val=1&cmd=0');" ontouchstart="fetch(document.location.origin+'/control?var=move&val=1&cmd=0');">FORWARD</button></td>
                        <td align="center"><button class="button" id="forward" onmousedown="fetch(document.location.origin+'/control?var=move&val=3&cmd=0');" ontouchstart="fetch(document.location.origin+'/control?var=move&val=3&cmd=0');">FORWARD END</button></td>
                        <td></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="button button2" id="turnleft" onmousedown="fetch(document.location.origin+'/control?var=move&val=2&cmd=0');" ontouchstart="fetch(document.location.origin+'/control?var=move&val=2&cmd=0');" onmouseup="fetch(document.location.origin+'/control?var=move&val=6&cmd=0');" ontouchend="fetch(document.location.origin+'/control?var=move&val=6&cmd=0');">LEFT</button></td>
                        <td align="center"><button class="button button2" id="steady" onclick="fetch(document.location.origin+'/control?var=funcMode&val=1&cmd=0');">STEADY</button></td>
                        <td align="center"><button class="button button2" id="turnright" onmousedown="fetch(document.location.origin+'/control?var=move&val=4&cmd=0');" ontouchstart="fetch(document.location.origin+'/control?var=move&val=4&cmd=0');" onmouseup="fetch(document.location.origin+'/control?var=move&val=6&cmd=0');" ontouchend="fetch(document.location.origin+'/control?var=move&val=6&cmd=0');">RIGHT</button></td>
                    </tr>
                    <tr>
                        <td></td>
                        <td align="center"><button class="button button2" id="backward" onmousedown="fetch(document.location.origin+'/control?var=move&val=5&cmd=0');" ontouchstart="fetch(document.location.origin+'/control?var=move&val=5&cmd=0');" onmouseup="fetch(document.location.origin+'/control?var=move&val=3&cmd=0');" ontouchend="fetch(document.location.origin+'/control?var=move&val=3&cmd=0');">REVERSE</button></td>
                        <td></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="button button4" id="stayLow" onclick="fetch(document.location.origin+'/control?var=funcMode&val=2&cmd=0');">StayLow</button></td>
                        <td align="center"><button class="button button4" id="handShake" onclick="fetch(document.location.origin+'/control?var=funcMode&val=3&cmd=0');">HandShake</button></td>
                        <td align="center"><button class="button button4" id="Jump" onclick="fetch(document.location.origin+'/control?var=funcMode&val=4&cmd=0');">Jump</button></td>
                    </tr>
                    <tr>
                        <td align="center"><button class="button button4" id="actionA" onclick="fetch(document.location.origin+'/control?var=funcMode&val=5&cmd=0');">ActionA</button></td>
                        <td align="center"><button class="button button4" id="actionB" onclick="fetch(document.location.origin+'/control?var=funcMode&val=6&cmd=0');">ActionB</button></td>
                        <td align="center"><button class="button button4" id="actionC" onclick="fetch(document.location.origin+'/control?var=funcMode&val=7&cmd=0');">ActionC</button></td>
                    </tr>
                    <tr><br><br><br></tr>
                    <tr>
                        <td align="center"><button class="button button4" id="initPos" onclick="fetch(document.location.origin+'/control?var=funcMode&val=8&cmd=0');">InitPos</button></td>
                        <td align="center"></td>
                        <td align="center"><button class="button button4" id="middlePos" onclick="fetch(document.location.origin+'/control?var=funcMode&val=9&cmd=0');">MiddlePos</button></td>
                    </tr>

                    
                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=AdjustFBcom&val=-1&cmd=0');">FB_COM -</button></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=AdjustFBcom&val=1&cmd=0');">FB_COM +</button></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=DiceAdjustFBcom&val=-1&cmd=0');">0.1 FB_COM -</button></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=DiceAdjustFBcom&val=1&cmd=0');">0.1 FB_COM +</button></td>
                    </tr>

                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=AdjustLRcom&val=-1&cmd=0');">LR_COM -</button></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=AdjustLRcom&val=1&cmd=0');">LR_COM +</button></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=DiceAdjustLRcom&val=-1&cmd=0');">0.1 LR_COM -</button></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=DiceAdjustLRcom&val=1&cmd=0');">0.1 LR_COM +</button></td>
                    </tr>

                    <td align="center"><input type="text" id="userValue" placeholder="Enter a value">
                    <td align="center"><button class="button button4" id="stayLow" onclick="TrotsetValAndFetch();">Free Trot</button>
                    
                    </td>
                    
                    <script>
                    function TrotsetValAndFetch() {
                        // Get the value from the input field
                        var userValue = document.getElementById('userValue').value;
                        
                        // Check if the user entered a valid number
                        if (!isNaN(userValue) && userValue.trim() !== "") {
                            // Fetch request with the user-provided value
                            fetch(document.location.origin + '/control?var=freetrot&val=' + userValue + '&cmd=0')
                                .then(response => response.json())
                                .then(data => console.log(data))
                                .catch(error => console.error('Error:', error));
                        } else {
                            alert("Invalid input. Please enter a number.");
                        }
                    }
                    </script>

                    <td align="center"><input type="text" id="clearanceValue" placeholder="Enter a value">
                    <td align="center"><button class="button button4" id="stayLow" onclick="WALKHEIGHTValAndFetch();">WALK HEIGHT</button>
                    
                    </td>
                    
                    <script>
                    function WALKHEIGHTValAndFetch() {
                        // Get the value from the input field
                        var userValue = document.getElementById('clearanceValue').value;
                        
                        // Check if the user entered a valid number
                        if (!isNaN(userValue) && userValue.trim() !== "") {
                            // Fetch request with the user-provided value
                            fetch(document.location.origin + '/control?var=walkheight&val=' + userValue + '&cmd=0')
                                .then(response => response.json())
                                .then(data => console.log(data))
                                .catch(error => console.error('Error:', error));
                        } else {
                            alert("Invalid input. Please enter a number.");
                        }
                    }
                    </script>


                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=imuswitch&val=1&cmd=0');">IMU ON</button></td>
                        <td align="center"></td>                        
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=imuswitch&val=0&cmd=0');">IMU OFF</button></td>
                    </tr>


                    <td align="center"><input type="text" id="kppitchValue" placeholder="Enter a value">
                    <td align="center"><button class="button button4" id="stayLow" onclick="KPPitchValAndFetch();">KP Pitch * 0.001</button>
                    </td>
                    <script>
                    function KPPitchValAndFetch() {
                        var userValue = document.getElementById('kppitchValue').value;
                        if (!isNaN(userValue) && userValue.trim() !== "") {
                            fetch(document.location.origin + '/control?var=kppitch&val=' + userValue + '&cmd=0')
                                .then(response => response.json())
                                .then(data => console.log(data))
                                .catch(error => console.error('Error:', error));
                        } else {
                            alert("Invalid input. Please enter a number.");
                        }
                    }
                    </script>

                    <td align="center"><input type="text" id="kprollValue" placeholder="Enter a value">
                    <td align="center"><button class="button button4" id="stayLow" onclick="KPRollValAndFetch();">KP Roll * 0.001</button>
                    </td>
                    <script>
                    function KPRollValAndFetch() {
                        var userValue = document.getElementById('kprollValue').value;
                        if (!isNaN(userValue) && userValue.trim() !== "") {
                            fetch(document.location.origin + '/control?var=kproll&val=' + userValue + '&cmd=0')
                                .then(response => response.json())
                                .then(data => console.log(data))
                                .catch(error => console.error('Error:', error));
                        } else {
                            alert("Invalid input. Please enter a number.");
                        }
                    }
                    </script>

                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=OPT&val=1&cmd=0');">Once_Pitch_threshold + </button></td>
                        <td align="center"></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=OPT&val=-1&cmd=0');">Once_Pitch_threshold - </button></td>
                    </tr>

                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=ORT&val=1&cmd=0');">Once_Roll_threshold + </button></td>
                        <td align="center"></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=ORT&val=-1&cmd=0');">Once_Roll_threshold - </button></td>
                    </tr>

                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=APT&val=1&cmd=0');">Adjust_Pitch_threshold + </button></td>
                        <td align="center"></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=APT&val=-1&cmd=0');">Adjust_Pitch_threshold - </button></td>
                    </tr>

                    <tr>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=ART&val=1&cmd=0');">Adjust_Roll_threshold + </button></td>
                        <td align="center"></td>
                        <td align="center"><button class="button" onclick="fetch(document.location.origin+'/control?var=ART&val=-1&cmd=0');">Adjust_Roll_threshold - </button></td>
                    </tr>


                </table>
            </div>
        </section>
    </section>
    <script>
    document.addEventListener('DOMContentLoaded', function(){
        var baseHost = document.location.origin
        var streamUrl = baseHost + ':81'

        const hide = el => {
            el.classList.add('hidden')
        }

        const show = el => {
            el.classList.remove('hidden')
          }

        const view = document.getElementById('stream')
        const viewContainer = document.getElementById('stream-container')
        const streamButton = document.getElementById('toggle-stream')
        const closeButton = document.getElementById('close-stream')

        const stopStream = () => {
            window.stop();
            streamButton.innerHTML = 'Start'
        }

        const startStream = () => {
            view.src = `${streamUrl}/stream`
            show(viewContainer)
            streamButton.innerHTML = 'Stop'
        }

        streamButton.onclick = () => {
            const streamEnabled = streamButton.innerHTML === 'Stop'
            if (streamEnabled) {
                stopStream()
            } else {
                startStream()
            }
        }
    });
    </script>
</body>

</html>

)rawliteral";
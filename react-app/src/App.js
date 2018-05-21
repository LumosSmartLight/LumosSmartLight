import React, { Component } from 'react';
import logo from '../src/lumos.png';
import './App.css';
import firebase from 'firebase'
import Slider from 'material-ui/Slider';
import Toggle from 'material-ui/Toggle';
import RaisedButton from 'material-ui/RaisedButton';
import { RadioButton, RadioButtonGroup } from 'material-ui/RadioButton';
import { config } from './config/firebase'
import { Card } from 'material-ui';
import { Line } from 'react-chartjs-2'
import moment from 'moment'

class App extends Component {
  constructor(props){
    super(props)
    this.state = {
      intensity: 0,
      mode: 1,
      isToggled: true,
      colors: {
          r: 0,
          g: 0,
          b: 255
      },
      isLoaded: false,
      chartData: {
        labels: [],
        datasets: [{
            label: 'Luminosidade',
            fill: false,
            lineTension: 0.1,
            backgroundColor: 'rgba(230, 90, 0, 0.4)',
            borderColor: 'rgb(230, 90, 0)',
            borderCapStyle: 'butt',
            borderDash: [],
            borderDashOffset: 0.0,
            borderJoinStyle: 'miter',
            pointBorderColor: 'rgb(230, 90, 0)',
            pointBackgroundColor: '#fff',
            pointBorderWidth: 1,
            pointHoverRadius: 5,
            pointHoverBackgroundColor: 'rgb(230, 90, 0)',
            pointHoverBorderColor: 'rgba(220,220,220,1)',
            pointHoverBorderWidth: 2,
            pointRadius: 1,
            pointHitRadius: 10,
            data: []
        }]
      }
    }

    this.chartOptions = {
      options: {
        responsive: true,
        title: {
          display: false,
          text: 'Chart.js Line Chart'
        },
        tooltips: {
          mode: 'index',
          intersect: false,
        },
        hover: {
          mode: 'nearest',
          intersect: true
        },
        scales: {
          xAxes: [{
            display: true,            
            type: 'time',
            time: {
              unit: 'minute',
            }
          }],
          yAxes: [{
            display: true,
            scaleLabel: {
              display: true,
              labelString: 'Luminosidade'
            }
          }]
        }
      }
    }
  
  }

  

  componentWillMount() {
    firebase.initializeApp(config)
    firebase.database().ref('luminosity').on('value', this.intensityStreamHandler)
    firebase.database().ref('config').once('value', (snap) => {
      let v =  snap.val()
      console.log(v)
      this.setState({
        isToggled : Boolean(v.ison), 
        intensity : parseInt(100 * v.intensity / 1024),
        mode: v.mode,
        colors: {
          r: v.colors.r,
          g: v.colors.g,
          b: v.colors.b
        },
        isLoaded: true
      })
    })
  }

  intensityStreamHandler = async (snapshot) => {
    console.log('new intensity data received')
    if (snapshot.val() === undefined || snapshot.val() === null) {
      return
    }

    let newChartData = {...this.state}
    let dataArray = Object.values(snapshot.val()).map((val) => {
      return {t: val.timestamp, y: val.value}
    })

    let labels = []
    for (let i = 0; i < dataArray.length; i++) {
      labels.push(parseInt(dataArray[i].t))
    }

    let intensity = parseInt((dataArray[dataArray.length - 1].y - 10) / 3)
    let value = (100 - intensity) > 0 ? (100 - intensity) : 0

    newChartData.chartData.labels = labels
    newChartData.chartData.datasets[0].data = dataArray

    this.setState(newChartData)
  }

  setIntensity = async (value) => {
    value = (value / 100) * 1024
    firebase.database().ref('config/intensity').set(parseInt(value))
  } 

  onIntensitySlide = (e, value) => {
    this.setState({intensity: value})
  }
  
  onIntensitySlideStop = () => {
    this.setIntensity(this.state.intensity)
  }

  // Todo: better color slide implementation
  onColorRedSlide = (e, value) => {
    this.setState({colors: {r: value, g: this.state.colors.g, b: this.state.colors.b}})
  }
  onColorGreenSlide = (e, value) => {
    this.setState({colors: {r: this.state.colors.r, g: value, b: this.state.colors.b}})
  }
  onColorBlueSlide = (e, value) => {
    this.setState({colors: {r: this.state.colors.r, g: this.state.colors.g, b: value}})
  }
  
  onColorSlideStop = () => {
    firebase.database().ref('config/colors').set(this.state.colors)
  }

  onToggle = (e) => {
    this.setState({ isToggled: !this.state.isToggled }, () => {
      firebase.database().ref('config/ison').transaction((v) => !v)
    })
  }

  onModeChange = (e, v) => {
    this.setState({ mode: v }, () => {
      firebase.database().ref('config/mode').set(v)
    })
  }

  onClearClick = (e) => {
    firebase.database().ref('luminosity').remove()
  }

  render() {
    const getColor = () => {
        let value = this.state.intensity/100
        //value from 0 to 1
        var hue = ((1 - value) * 120).toString(10);
        return ["hsl(", hue, ",100%,45%)"].join("");    
    }

    const styles = {
      block: {
        maxWidth: 250,
      },
      radioButton: {
        marginBottom: 16,
      },
      toggle: {
        marginBottom: 16,
      },
      blueslider: {
        selectionColor: '#00FFFF',
        handleFillColor: '#00FFFF'
      }
    };

    return (
      <div className="App">
        <header className="App-header">
          <img className='App-logo' src={logo}/>
        </header>

        <Card className='card' style={{opacity: this.state.isLoaded ? 1 : 0}}>
          <div className='slider-container'>
            <h1 style={{ textAlign: 'left', fontSize: 22 }}> Intensity <span className='intensity' style={{ color: getColor() }}> {parseInt(this.state.intensity)}% </span>
              <Toggle className='toggleOnOff'
                style={styles.toggle}
                toggled={this.state.isToggled}
                defaultToggled={true}
                onToggle={this.onToggle}
              />
            </h1>
            <Slider name="intensity" step={1} min={0} max={100} value={this.state.intensity} onChange={this.onIntensitySlide} onDragStop={this.onIntensitySlideStop} />

            <h1 style={{ textAlign: 'left', fontSize: 22 }}> RGB </h1>
            <Slider sliderStyle={styles.blueslider} name="red" step={1} min={0} max={255} value={this.state.colors.r} onChange={this.onColorRedSlide} onDragStop={this.onColorSlideStop} />
            <Slider name="green" step={1} min={0} max={255} value={this.state.colors.g} onChange={this.onColorGreenSlide} onDragStop={this.onColorSlideStop.bind(this, "g")} />
            <Slider name="blue" step={1} min={0} max={255} value={this.state.colors.b} onChange={this.onColorBlueSlide} onDragStop={this.onColorSlideStop.bind(this, "b")} />

            <RadioButtonGroup name="lightmode" defaultSelected={this.state.mode} valueSelected={this.state.mode} onChange={this.onModeChange}>
              <RadioButton
                value={0}
                label="Manual"
                style={styles.radioButton}
              />
              <RadioButton
                value={1}
                label="Automático"
                style={styles.radioButton}
              />
              <RadioButton
                value={2}
                label="Modo Dormir"
                style={styles.radioButton}
              />
            </RadioButtonGroup>
          </div>
        </Card>

        <Card className='card' style={{ opacity: this.state.isLoaded ? 1 : 0 }}>
          <Line data={this.state.chartData} options={this.chartOptions.options}/>
          <RaisedButton label="Clear history" onClick={this.onClearClick} />
        </Card>
        
      </div>
    );
  }
}

export default App;

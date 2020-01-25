using System;
using System.IO.Ports;
using System.Reactive.Linq;
using System.Text;
using System.Windows.Forms;
using XInputDotNetPure;

namespace _6_dof_tester
{
    public partial class Form1 : Form
    {
        PlatformInterface s;
        public Form1()
        {
            InitializeComponent();
            s = new PlatformInterface();
            LoadComs();
            resetvalues();
        }

        String result;

        private string LoadComs()
        {
            comboBox1.Items.Clear();
            string[] ports = SerialPort.GetPortNames();
            comboBox1.Items.AddRange(ports);
            
            return null;
        }

        void _serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            byte[] buffer = new byte[s._serialPort.ReadBufferSize];
            int bytesRead = 0;

            try
            {
                bytesRead = s._serialPort.Read(buffer, 0, buffer.Length);
            }
            catch (Exception)
            {
                
            }

            String tString = Encoding.ASCII.GetString(buffer, 0, bytesRead);
            result += tString;
        }
 
        public void InvokeTextBox(TextBox textbox, string value)
        {
            if (InvokeRequired)
            {       
                this.BeginInvoke(new Action<TextBox,string>(InvokeTextBox), new object[] { textbox, value });
                return;
            }
            
            textbox.Text = value;

        }  

        private void button5_Click(object sender, EventArgs e)
        {
            textBox7.Clear();
            updateDemoPosition();
        }

        private void updateDemoPosition()
        {
            int x;
            int y;
            int z;
            int rotx;
            int roty;
            int rotz;

            int.TryParse(textBox1.Text, out x);
            int.TryParse(textBox2.Text, out y);
            int.TryParse(textBox3.Text, out z);
            int.TryParse(textBox4.Text, out rotx);
            int.TryParse(textBox5.Text, out roty);
            int.TryParse(textBox6.Text, out rotz);

            int[] arr = { x, y, z, rotx, roty, rotz };
            s.SetPositions(arr);
        }

        private void button8_Click(object sender, EventArgs e)
        {
            resetvalues();
        }

        private void resetvalues()
        {
            //0 - 1023
            int defaultVal = 2047;

            InvokeTextBox(textBox1, defaultVal.ToString());
            InvokeTextBox(textBox2, defaultVal.ToString());
            InvokeTextBox(textBox3, defaultVal.ToString());
            InvokeTextBox(textBox4, defaultVal.ToString());
            InvokeTextBox(textBox5, defaultVal.ToString());
            InvokeTextBox(textBox6, defaultVal.ToString());
        }

        IDisposable demoMoverTimer;

        private void button1_Click_1(object sender, EventArgs e)
        {
            IObservable<long> timer = Observable.Timer(TimeSpan.FromMilliseconds(10), TimeSpan.FromMilliseconds(10));
            demoMoverTimer = timer.Subscribe(x => {
                try
                {
                    GamePadState state = GamePad.GetState(PlayerIndex.One);
                    int xVal = (int)ExtensionMethods.Map((decimal)state.ThumbSticks.Left.Y, -1, 1, 0, 4094);
                    int yVal = (int)ExtensionMethods.Map(-(decimal)state.ThumbSticks.Left.X, -1, 1, 0, 4094);


                    int rightz = (int)ExtensionMethods.Map((decimal)state.Triggers.Right, 0, 1, 2047, 4094);
                    int leftz = (int)ExtensionMethods.Map((decimal)state.Triggers.Left, 0, 1, 2047, 4094);

                    int zVal = -rightz + leftz + 2047;

                    int xRotVal = (int)ExtensionMethods.Map(-(decimal)state.ThumbSticks.Right.Y, -1, 1, 0, 4094);
                    int yRotVal = (int)ExtensionMethods.Map((decimal)state.ThumbSticks.Right.X, -1, 1, 0, 4094);

                    int rightZRot = 0;
                    if (state.Buttons.RightShoulder == XInputDotNetPure.ButtonState.Pressed)
                    {
                        rightZRot = 2000;
                    }

                    int leftZRot = 0;
                    if (state.Buttons.LeftShoulder == XInputDotNetPure.ButtonState.Pressed)
                    {
                        leftZRot = 2000;
                    }


                    // int rightZRot = (int)ExtensionMethods.Map((decimal)state.Buttons.RightShoulder, 0, 1, 127, 255);
                    // int leftZRot = (int)ExtensionMethods.Map((decimal)state.Buttons.LeftShoulder, 0, 1, 127, 255);

                    int ZRotVal = rightZRot + -leftZRot + 2047;


                    InvokeTextBox(textBox1, xVal.ToString());
                    InvokeTextBox(textBox2, yVal.ToString());
                    InvokeTextBox(textBox3, zVal.ToString());
                    InvokeTextBox(textBox4, xRotVal.ToString());
                    InvokeTextBox(textBox5, yRotVal.ToString());
                    InvokeTextBox(textBox6, ZRotVal.ToString());

                    updateDemoPosition();
                }
                catch (Exception ex)
                {
                    string te = "";
                }
            });
           
        }

        private void button2_Click_1(object sender, EventArgs e)
        {
            if(demoMoverTimer != null)
                demoMoverTimer.Dispose();
        }

        private void button3_Click_1(object sender, EventArgs e)
        {
            if (s.Open(comboBox1.SelectedItem.ToString()))
            {
                s._serialPort.DataReceived += new SerialDataReceivedEventHandler(_serialPort_DataReceived);
            }
            else
            {
                MessageBox.Show("Error Opening");
            }

        }

        private void button4_Click_1(object sender, EventArgs e)
        {
            s.disconnect();
        }

        private void button9_Click(object sender, EventArgs e)
        {
            textBox7.Clear();
            result = "";
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            s.disconnect();
        }
    }


    public class PlatformInterface
    {
     
        public SerialPort _serialPort = new SerialPort();
        private int _baudRate = 115200;
       
        private int _dataBits = 8;
        private Handshake _handshake = Handshake.None;
        private Parity _parity = Parity.None;
        private string _portName = "COM8";
        private StopBits _stopBits = StopBits.One;
        private string tString = string.Empty;

        public bool Open(String com)
        {
            try
            {
                _portName = com;

                _serialPort.BaudRate = _baudRate;
                _serialPort.DataBits = _dataBits;
                _serialPort.Handshake = _handshake;
                _serialPort.Parity = _parity;
                _serialPort.PortName = _portName;
                _serialPort.StopBits = _stopBits;
           
                _serialPort.Open();

                String t = "";
            }
            catch { return false; }
            return true;
        }

        public void WriteAction(String val)
        {
            try
            {
                _serialPort.Write(val);
            }
            catch (Exception)
            {
                
            }
        }
        
        public void SetPositions(int[] values)
        {
            String command = "";
           
            for (int i = 0; i < 5; i++)
            {
                float value = values[i];

                command += value.ToString();
                command += ",";
            }

            command += values[5].ToString();

            command += "X\n";

            this.WriteAction(command);
        }

        internal void disconnect()
        {
            try
            {
                _serialPort.Close();
            }
            catch {}
        }
    }
}

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

using System.Net;
using System.Net.Sockets;
using System.Windows.Forms.DataVisualization.Charting;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;
using System.Threading;



namespace GUI_Interface
{
    public partial class Form1 : Form
    {

        private UdpClient sendClient;
        private UdpClient client;
        private int receivePort; // Port de réception dynamique
      //  private string data = "";
        private bool isReceiving = false; // Flag to check if receiving is active

        private int selectedValue;
       




        public Form1()
        {
            InitializeComponent();
            dataGridView1.Visible = true;
            chart_Udc.Visible = false;
            chart_Idc.Visible = false;
            Phase_shift.ValueChanged += trackBar1_ValueChanged;

            // Initialize the Timer
            
            timer1.Interval = 500; // Set the interval to 500 ms (0.5 seconds)
            timer1.Tick += Timer_Tick;
        }

        private void button_Start_Click(object sender, EventArgs e)
        {
            try
            {
                // Récupérer le port d'écoute depuis textBox_ReceivePort
                if (!int.TryParse(textBox_Receive_Port.Text, out receivePort))
                {
                    MessageBox.Show("Invalid port number.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    return;
                }

                // Envoyer le message START avec le client d'envoi
                SendUdpMessage("192.168.0.4", 28000, "START");

                // Fermer le client d'envoi après l'envoi du message START
                if (sendClient != null)
                {
                    sendClient.Close();
                    sendClient = null;
                }

                richTextBox1.Text += "Starting to receive...\n"; // Debug

                // Fermer l'ancien client si nécessaire
                if (client != null)
                {
                    client.Close();
                    client = null;
                    MessageBox.Show("Fermeture client!", "Success", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }

                // Créer un nouveau UdpClient avec le port spécifié pour la réception
                client = new UdpClient(receivePort);
                isReceiving = true; // La réception commence
                timer1.Start(); // Start the timer
                client.BeginReceive(new AsyncCallback(ReceiveCallback), null);
                richTextBox1.Text += $"Listening on port: {receivePort}\n"; // Debug
            }
            catch (Exception ex)
            {
                richTextBox1.Text += "Error: " + ex.Message.ToString();
            }
        }

        private void button_Stop_Click(object sender, EventArgs e)
        {
            try
            {
                // Fermer le client de réception avant d'envoyer le message STOP
                if (client != null)
                {
                    isReceiving = false; // La réception s'arrête
                    timer1.Stop(); // Stop the timer
                    client.Close();
                    client = null;
                    richTextBox1.Text += "Stopped receiving.\n"; // Debug
                }

                // Ouvrir un nouveau client d'envoi avec le même port que le client de réception
                sendClient = new UdpClient(receivePort);

                // Envoyer le message STOP avec le client d'envoi
                SendUdpMessage("192.168.0.4", 28000, "STOP");

                // Fermer le client d'envoi après l'envoi du message STOP
                if (sendClient != null)
                {
                    sendClient.Close();
                    sendClient = null;
                }
            }
            catch (Exception ex)
            {
                richTextBox1.Text += "Error: " + ex.Message.ToString();
            }
        }

        private void SendUdpMessage(string ipAddress, int port, string message)
        {
            try
            {
                if (IPAddress.TryParse(ipAddress, out IPAddress address) && port > 0 && port <= 65535)
                {
                    // Utiliser le même port que le client de réception pour envoyer des messages
                    if (sendClient == null)
                    {
                        sendClient = new UdpClient(receivePort);
                    }

                    sendClient.Connect(address, port);
                    byte[] sendBytes = Encoding.ASCII.GetBytes(message);
                    sendClient.Send(sendBytes, sendBytes.Length);
                  //  richTextBox2.Text += "Message sent successfully!";
                }
                else
                {
                    richTextBox2.Text += "Invalid IP address or port.";
                }
            }
            catch (Exception ex)
            {
                richTextBox2.Text += "An error occurred: {ex.Message}" + ex.Message.ToString(); ;
            }
        }

        private void trackBar1_ValueChanged(object sender, EventArgs e)
        {
            if (isReceiving)
            {
                textBox_Phase_shift.Clear();
                selectedValue = Phase_shift.Value;      
                textBox_Phase_shift.Text += $"value: {selectedValue}\n"; // Debug message
                
            }
        }

        private void Timer_Tick(object sender, EventArgs e)
        {
            if (isReceiving)
            {
                string message = selectedValue + "";
                // Fermer le client de réception avant d'envoyer le message STOP
                if (client != null)
                {
                    isReceiving = false; // La réception s'arrête
                    client.Close();
                    client = null;
                  
                }

                // Ouvrir un nouveau client d'envoi avec le même port que le client de réception
                sendClient = new UdpClient(receivePort);
                //  string message = selectedValue.ToString(); // Convert the value to a string
               SendUdpMessage("192.168.0.4", 28000, message);

                // Fermer le client d'envoi après l'envoi du message STOP
                if (sendClient != null)
                {
                    sendClient.Close();
                    sendClient = null;
                }
                client = new UdpClient(receivePort);
                isReceiving = true; // La réception commence
                client.BeginReceive(new AsyncCallback(ReceiveCallback), null);      
            }
        }


            private void PlotChart()
        {
           
            chart_Udc.Series.Clear();
            chart_Idc.Series.Clear();

            /*************************************************************Tension*****************************/
            // Créer une nouvelle série
            Series series1 = new Series
            {
                Name = "Series1",
                Color = Color.Blue,
                IsVisibleInLegend = false,
                ChartType = SeriesChartType.Line
            };

            chart_Udc.Series.Add(series1);
         

            // Ajouter des points à la série à partir des colonnes 0 et 1 du DataGridView
            for (int i = 0; i < dataGridView1.Rows.Count; i++)
            {
                if (dataGridView1.Rows[i].Cells[0].Value != null && dataGridView1.Rows[i].Cells[1].Value != null)
                {
                    double xValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[0].Value);
                    double yValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[1].Value);
                    series1.Points.AddXY(xValue, yValue);
                   // chart_Udc.Series[0].Points.AddXY(xValue, yValue);
                 
                }
            }

            // Créer une série pour la courbe colonne 3 par rapport à la colonne de base
            Series series2 = new Series
            {
                Name = "Series2",
                Color = Color.Red,
                IsVisibleInLegend = true,
                ChartType = SeriesChartType.Line
            };
            chart_Udc.Series.Add(series2);

            // Ajouter des points à la série2 à partir des colonnes 0 (X) et 3 (Y) du DataGridView
            for (int i = 0; i < dataGridView1.Rows.Count; i++)
            {
                if (dataGridView1.Rows[i].Cells[0].Value != null && dataGridView1.Rows[i].Cells[3].Value != null)
                {
                    double xValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[0].Value);
                    double yValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[3].Value);
                    series2.Points.AddXY(xValue, yValue);
                }
            }

            /***********************************Courant***************************************/

            Series series3 = new Series
            {
                Name = "Series1",
                Color = Color.Blue,
                IsVisibleInLegend = false,
                ChartType = SeriesChartType.Line
            };

            chart_Idc.Series.Add(series3);


            // Ajouter des points à la série à partir des colonnes 0 et 1 du DataGridView
            for (int i = 0; i < dataGridView1.Rows.Count; i++)
            {
                if (dataGridView1.Rows[i].Cells[0].Value != null && dataGridView1.Rows[i].Cells[2].Value != null)
                {
                    double xValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[0].Value);
                    double yValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[2].Value);
                    series3.Points.AddXY(xValue, yValue);
                    // chart_Udc.Series[0].Points.AddXY(xValue, yValue);

                }
            }

            // Créer une série pour la courbe colonne 3 par rapport à la colonne de base
            Series series4 = new Series
            {
                Name = "Series2",
                Color = Color.Red,
                IsVisibleInLegend = true,
                ChartType = SeriesChartType.Line
            };
            chart_Idc.Series.Add(series4);

            // Ajouter des points à la série2 à partir des colonnes 0 (X) et 3 (Y) du DataGridView
            for (int i = 0; i < dataGridView1.Rows.Count; i++)
            {
                if (dataGridView1.Rows[i].Cells[0].Value != null && dataGridView1.Rows[i].Cells[4].Value != null)
                {
                    double xValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[0].Value);
                    double yValue = Convert.ToDouble(dataGridView1.Rows[i].Cells[4].Value);
                    series4.Points.AddXY(xValue, yValue);
                }
            }


            // Configurer les axes du graphique
            chart_Udc.ChartAreas[0].AxisX.Title = "Column 1";
            chart_Udc.ChartAreas[0].AxisY.Title = "Value";
            chart_Idc.ChartAreas[0].AxisX.Title = "Column 1";
            chart_Idc.ChartAreas[0].AxisY.Title = "Value";
        }


        private void ReceiveCallback(IAsyncResult result)
        {
            if (!isReceiving)
            {
                return; // Si la réception a été arrêtée, ne rien faire
            }

            try
            {
                IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
                byte[] received = client.EndReceive(result, ref remoteEndPoint);

               // Log the length of the received byte array
                this.Invoke(new MethodInvoker(delegate
                {
                    richTextBox2.Text += $"Received byte length: {received.Length}\n";
                }));
                // Assumer que les données reçues sont des uint32_t
               // Assumer que les données reçues sont des uint16_t
        int numberOfUint16 = received.Length / sizeof(ushort);
        ushort[] dataArray = new ushort[numberOfUint16];

        for (int i = 0; i < numberOfUint16; i++)
        {
            dataArray[i] = BitConverter.ToUInt16(received, i * sizeof(ushort));
        }
              
                // Ajouter les données au DataGridView
                this.Invoke(new MethodInvoker(delegate
                {
                    dataGridView1.Rows.Clear(); // Effacer les lignes existantes si nécessaire

                    int numberOfColumns = 5;
                    int rowsPerColumn = 100;

                    // Ajouter des lignes au DataGridView pour contenir toutes les données
                    for (int i = 0; i < rowsPerColumn; i++)
                    {
                        dataGridView1.Rows.Add();
                    }

                    for (int col = 0; col < numberOfColumns; col++)
                    {
                        for (int row = 0; row < rowsPerColumn; row++)
                        {
                            int dataIndex = col * rowsPerColumn + row;
                            if (dataIndex < dataArray.Length)
                            {
                                dataGridView1.Rows[row].Cells[col].Value = dataArray[dataIndex];
                            }
                        }
                    }
                    richTextBox2.Text += $"value: {dataArray.Length}\n";
                    PlotChart();
                }));

                // Recommencer à recevoir
                client.BeginReceive(new AsyncCallback(ReceiveCallback), null);
                
            }
            catch (Exception ex)
            {
                if (isReceiving)
                {
                    this.Invoke(new MethodInvoker(delegate
                    {
                        richTextBox1.Text += "\nError: " + ex.Message.ToString();
                    }));
                }
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            try
            {
                // Assurer la fermeture propre du UdpClient
                isReceiving = false;
                client?.Close();
                sendClient?.Close(); // Fermer le client d'envoi ici
                base.OnFormClosing(e);
            }
            catch (Exception ex)
            {
                richTextBox1.Text += $"Error closing: {ex.Message}";
            }
        }

        private void tableauToolStripMenuItem_Click(object sender, EventArgs e)
        {
            dataGridView1.Visible = true;
            chart_Udc.Visible = false;
            chart_Idc.Visible = false;
        }

        private void schemaToolStripMenuItem_Click(object sender, EventArgs e)
        {

            dataGridView1.Visible = false;
            chart_Udc.Visible = true;
            chart_Idc.Visible = false;
            PlotChart(); // Rafraîchir le graphique quand il devient visible
        }

        private void courantToolStripMenuItem_Click(object sender, EventArgs e)
        {
            dataGridView1.Visible = false;
            chart_Udc.Visible = false;
            chart_Idc.Visible = true;
            PlotChart(); // Rafraîchir le graphique quand il devient visible
        }
    }
}
// Convertir le tableau en une chaîne de caractères
/*             string dataString = string.Join(", ", dataArray);

             // Utiliser MethodInvoker pour éviter le cross-threading
             this.Invoke(new MethodInvoker(delegate
             {
                 richTextBox2.Text += $"\nReceived data from {remoteEndPoint.Address}:{remoteEndPoint.Port}: {dataString}";
             }));

// Ajouter les données au DataGridView
this.Invoke(new MethodInvoker(delegate
{
    dataGridView1.Rows.Clear(); // Effacer les lignes existantes si nécessaire

    for (int i = 0; i < dataArray.Length; i++)
    {
        dataGridView1.Rows.Add(i, dataArray[i]); // Ajouter chaque élément dans une nouvelle ligne du DataGridView
    }
}));
*/
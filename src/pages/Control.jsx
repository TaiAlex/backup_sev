import { Button } from "@material-tailwind/react";
import React, { useEffect, useState, useRef } from "react";
// import axios from "axios";
import axios from "./checkToken";
import { Line, Bar } from "react-chartjs-2";

import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  BarElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";

ChartJS.register(
  CategoryScale,
  LinearScale,
  BarElement,
  Title,
  Tooltip,
  Legend
);
const MAX_CLICKS = 4;
const LOCK_DURATION = 60 * 1000;
const DOUBLE_CLICK_THRESHOLD = 300;
const GetDataTime = 0.2 * 60;
const Control = () => {
  const [isChecked, setIsChecked] = useState(false);
  const [Display, setDisplay] = useState("1");
  const [selector, setSelector] = useState("motor1");
  const [valueMotor1, setValueMotor1] = useState(false);
  const [valueMotor2, setValueMotor2] = useState(false);
  const [valueMotor3, setValueMotor3] = useState(false);
  const [data1, setData] = useState([]);
  const dataChart = data1.slice(-30);
  const [totalPump1, setTotalPump1] = useState();
  const [totalPump2, setTotalPump2] = useState();
  const [totalPump3, setTotalPump3] = useState();
  const [startThreshold, setStartThreshold] = useState(50);
  const [stopThreshold, setStopThreshold] = useState(50);

  const [pumps, setPumps] = useState([false, false, false]);
  const [clickCount, setClickCount] = useState(0);
  const [isLocked, setIsLocked] = useState(false);
  const [timeLeft, setTimeLeft] = useState(0);

  const lastClickTimeRef = useRef(0);
  const [Flag, setFlag] = useState();
  // Lấy token
  const token = localStorage.getItem("token");
  const access_token = "Bearer " + token;

  const handleStartThresholdChange = (event) => {
    setStartThreshold(event.target.value);
  };

  const handleStopThresholdChange = (event) => {
    setStopThreshold(event.target.value);
  };

  const handleSaveClick = async () => {
    console.log("Pump Start threshold saved:", startThreshold);
    console.log("Pump Stop threshold saved:", stopThreshold);
    const url = `http://34.87.151.244:1506/threshold/humi`;
    try {
      console.log("before send", url);
      const response = await axios.post(
        url,
        {
          attribute: "humi",
          upper: stopThreshold,
          lower: startThreshold,
        },
        {
          headers: {
            accept: "application/json",
            // "Content-Type": "application/json",
            Authorization: access_token,
          },
        }
      );

      console.log("Success:", response);
    } catch (error) {
      console.error("Error:", error);
    }
  };

  // Tải trạng thái từ localStorage khi component mount
  useEffect(() => {
    const storedLockStatus = localStorage.getItem("isLocked");
    const storedTimeLeft = localStorage.getItem("timeLeft");
    const storedClickCount = localStorage.getItem("clickCount");

    if (storedLockStatus === "true") {
      const timeRemaining = parseInt(storedTimeLeft, 10) - Date.now();
      if (timeRemaining > 0) {
        setIsLocked(true);
        setTimeLeft(timeRemaining);
      } else {
        localStorage.removeItem("isLocked");
        localStorage.removeItem("timeLeft");
        setIsLocked(false);
      }
    }

    if (storedClickCount) {
      setClickCount(parseInt(storedClickCount, 10));
    }
  }, []);

  // Lưu trạng thái vào localStorage khi thay đổi
  useEffect(() => {
    localStorage.setItem("clickCount", clickCount);
    if (isLocked) {
      localStorage.setItem("isLocked", "true");
      localStorage.setItem("timeLeft", (Date.now() + timeLeft).toString());
    } else {
      localStorage.removeItem("isLocked");
      localStorage.removeItem("timeLeft");
    }
  }, [clickCount, isLocked, timeLeft]);

  // Xử lý bộ đếm ngược
  useEffect(() => {
    let timer;
    if (isLocked && timeLeft > 0) {
      timer = setInterval(() => {
        setTimeLeft((prevTime) => prevTime - 1000);
      }, 1000);
    } else if (timeLeft <= 0) {
      setIsLocked(false);
      setClickCount(0);
    }
    return () => clearInterval(timer);
  }, [isLocked, timeLeft]);

  const handleClick = async (index) => {
    if (isLocked) return;
    console.log("before pump", pumps);

    const now = Date.now();
    const timeSinceLastClick = now - lastClickTimeRef.current;

    if (timeSinceLastClick <= DOUBLE_CLICK_THRESHOLD) {
      setClickCount((prev) => {
        const newClickCount = prev + 1;
        if (newClickCount >= MAX_CLICKS) {
          setIsLocked(true);
          setTimeLeft(LOCK_DURATION);
        }
        return newClickCount;
      });
    } else {
      setClickCount(1);
    }

    lastClickTimeRef.current = now;

    setPumps(pumps.map((pump, i) => (i === index ? !pump : pump)));

    // console.log("pump", index, "data", pumps[index]);
    setFlag(index + 1);
  };
  if (Flag) {
    console.log("after", pumps);

    //gửi control lên api
    const controlPanel = async () => {
      try {
        const url = `http://34.87.151.244:1506/control/motor/${Flag}`;
        console.log("before send", url);
        const response = await axios.post(
          url,
          {
            status: pumps[Flag - 1],
          },
          {
            headers: {
              accept: "application/json",
              Authorization: access_token,
            },
          }
        );

        console.log("Success:", response);
      } catch (error) {
        console.error("Error:", error);
      }
    };
    controlPanel();
    setFlag(false);
  }
  const formatTime = (ms) => {
    const minutes = Math.floor(ms / 60000);
    const seconds = Math.floor((ms % 60000) / 1000);
    return `${minutes}:${seconds < 10 ? "0" : ""}${seconds}`;
  };

  async function loadData() {
    const response = await axios.get("http://34.87.151.244:1506/api/motor/0", {
      headers: {
        Authorization: access_token,
        accept: "application/json",
        "Content-Type": "application/x-www-form-urlencoded",
      },
    });

    const dt1 = response.data;
    // console.log(dt1);
    setData(dt1);
    setValueMotor1(dt1.slice(-1)[0]["motor1"]);
    setValueMotor2(dt1.slice(-1)[0]["motor2"]);
    setValueMotor3(dt1.slice(-1)[0]["motor3"]);
  }
  const generateRandomValues = () => {
    setTotalPump1(Math.floor(Math.random() * 100) + 1);
    setTotalPump2(Math.floor(Math.random() * 100) + 1);
    setTotalPump3(Math.floor(Math.random() * 100) + 1);
  };
  useEffect(() => {
    const ciupezoi = setInterval(() => {
      loadData();
      generateRandomValues();
    }, 5000);
  }, 30000);

  const chartData = (lineColor, data_to_draw) => {
    const labels = dataChart.map((item) => item.time);
    const data = dataChart.map((item) => item[data_to_draw]);

    return {
      labels: labels,
      datasets: [
        {
          label: "",
          data: data, // Assuming data_CO2 has a co2 field
          borderColor: lineColor,
          backgroundColor: "rgba(75,192,192,0.2)",
          fill: false,
          stepped: true,
        },
      ],
    };
  };

  const options = (title) => ({
    responsive: true,
    plugins: {
      legend: {
        display: false,
      },
      title: {
        display: true,
        text: title,
      },
    },
  });
  // ve total pump

  const labels = ["Pump 1", "Pump 2", "Pump 3"];
  const data2 = {
    labels: labels,
    datasets: [
      {
        axis: "y",
        label: "Total in a Week",

        data: [totalPump1, totalPump2, totalPump3],
        fill: false,
        backgroundColor: [
          "rgba(75, 192, 192, 0.2)",
          "rgba(54, 162, 235, 0.2)",
          "rgba(153, 102, 255, 0.2)",
        ],
        borderColor: [
          "rgb(75, 192, 192)",
          "rgb(54, 162, 235)",
          "rgb(153, 102, 255)",
          // "rgb(201, 203, 207)",
        ],
        borderWidth: 1,
      },
    ],
  };
  const optionsBar = {
    indexAxis: "y",
    scales: {
      x: {
        beginAtZero: true,
        title: {
          display: true,
          text: "Volume (liters)", // Set the y-axis title with the unit
        },
      },
    },
    plugins: {
      legend: {
        labels: {
          fontSize: 15,
        },
      },
    },
  };
  console.log(localStorage.getItem("isChecked"));
  const ModeControl = async (e) => {
    setIsChecked(e.target.checked);
    localStorage.setItem("isChecked", e.target.checked);
    try {
      const response = await axios.post(
        "http://34.87.151.244:1506/control_mode",
        { mode: isChecked },
        {
          headers: {
            accept: "application/json",
            Authorization: access_token,
          },
        }
      );
    } catch (error) {
      console.error("Error:", error);
    }
  };

  return (
    <div className="flex  h-full w-full ">
      {/* -----------------------PC View----------------------- */}

      <div className="hidden sm:block w-full">
        <div className="flex p-4 h-full w-full gap-3  ">
          {/* ----------------------- View Control Panel----------------------- */}

          <div className="p-4 h-full w-2/3  bg-teal-100 rounded-3xl ">
            <h1 className="ml-4 font-bold text-center mb-4 ">PUMP VIEW</h1>
            <div className="flex left-1 gap-3 items-center justify-center">
              <div className=" flex flex-col  items-center h-1/6 w-36 p-2  shadow-xl bg-white text-black rounded-md mr-2 ">
                <div class="flex justify-between text-black">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                    class="icon icon-tabler icons-tabler-outline icon-tabler-engine"
                  >
                    <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                    <path d="M3 10v6" />
                    <path d="M12 5v3" />
                    <path d="M10 5h4" />
                    <path d="M5 13h-2" />
                    <path d="M6 10h2l2 -2h3.382a1 1 0 0 1 .894 .553l1.448 2.894a1 1 0 0 0 .894 .553h1.382v-2h2a1 1 0 0 1 1 1v6a1 1 0 0 1 -1 1h-2v-2h-3v2a1 1 0 0 1 -1 1h-3.465a1 1 0 0 1 -.832 -.445l-1.703 -2.555h-2v-6z" />
                  </svg>
                  <p className="text-left">
                    <b>Motor 1</b>
                  </p>
                </div>

                <div class="ml-2 w-full flex-1">
                  <div class="mt-1 text-base text-gray-600">
                    Status : <span>{valueMotor1 ? "ON" : "Off"}</span>
                  </div>
                  <div class="mt-1 text-base text-gray-600">Time : </div>
                  <div class="mt-1 text-base text-gray-600">Flow : </div>
                </div>
              </div>
              <div className=" flex flex-col  items-center h-1/6 w-36 p-2 shadow-xl bg-white text-black rounded-md mr-2 ">
                <div class="flex justify-between text-black">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                    class="icon icon-tabler icons-tabler-outline icon-tabler-engine"
                  >
                    <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                    <path d="M3 10v6" />
                    <path d="M12 5v3" />
                    <path d="M10 5h4" />
                    <path d="M5 13h-2" />
                    <path d="M6 10h2l2 -2h3.382a1 1 0 0 1 .894 .553l1.448 2.894a1 1 0 0 0 .894 .553h1.382v-2h2a1 1 0 0 1 1 1v6a1 1 0 0 1 -1 1h-2v-2h-3v2a1 1 0 0 1 -1 1h-3.465a1 1 0 0 1 -.832 -.445l-1.703 -2.555h-2v-6z" />
                  </svg>
                  <p className="text-left">
                    <b>Motor 2</b>
                  </p>
                </div>

                <div class="ml-2 w-full flex-1">
                  <div class="mt-1 text-base text-gray-600">
                    Status : <span>{valueMotor2 ? "ON" : "Off"}</span>
                  </div>
                  <div class="mt-1 text-base text-gray-600">Time : </div>
                  <div class="mt-1 text-base text-gray-600">Flow : </div>
                </div>
              </div>
              <div className=" flex flex-col  items-center h-1/6 w-36 p-2 shadow-xl bg-white text-black rounded-md mr-2 ">
                <div class="flex justify-between text-black">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                    class="icon icon-tabler icons-tabler-outline icon-tabler-engine"
                  >
                    <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                    <path d="M3 10v6" />
                    <path d="M12 5v3" />
                    <path d="M10 5h4" />
                    <path d="M5 13h-2" />
                    <path d="M6 10h2l2 -2h3.382a1 1 0 0 1 .894 .553l1.448 2.894a1 1 0 0 0 .894 .553h1.382v-2h2a1 1 0 0 1 1 1v6a1 1 0 0 1 -1 1h-2v-2h-3v2a1 1 0 0 1 -1 1h-3.465a1 1 0 0 1 -.832 -.445l-1.703 -2.555h-2v-6z" />
                  </svg>
                  <p className="text-left">
                    <b>Motor 3 </b>
                  </p>
                </div>

                <div class="ml-2 w-full flex-1">
                  <div class="mt-1 text-base text-gray-600">
                    Status : <span>{valueMotor3 ? "ON" : "Off"}</span>
                  </div>
                  <div class="mt-1 text-base text-gray-600">Time : </div>
                  <div class="mt-1 text-base text-gray-600">Flow : </div>
                </div>
              </div>
            </div>
            <div className=" h-2/3 w-full p-4">
              <div className="bg-white h-full w-full rounded-3xl">
                <ul className=" flex justify-center w-full overflow-x-auto ">
                  <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-2">
                    <span
                      class="ml-2 "
                      onClick={() => {
                        setSelector("motor1");
                      }}
                    >
                      MOTOR 1
                    </span>
                  </li>
                  <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-2">
                    <span
                      class="ml-2 "
                      onClick={() => {
                        setSelector("motor2");
                      }}
                    >
                      MOTOR 2
                    </span>
                  </li>
                  <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-2">
                    <span
                      class="ml-2 "
                      onClick={() => {
                        setSelector("motor3");
                      }}
                    >
                      MOTOR 3
                    </span>
                  </li>
                </ul>
                <div id="chart_Sensor_1" className="p-2  ">
                  <Line
                    data={chartData("#f15bb5", selector)}
                    options={options(selector)}
                  />
                </div>
              </div>
            </div>
          </div>
          {/* -----------------------End View Control Panel----------------------- */}

          {/* -----------------------Control Pump---------------------- */}

          <div className="p-4 h-full w-2/3 rounded-3xl bg-teal-100 ">
            <h1 className="ml-4 font-bold text-center mb-4"> Control Pump</h1>
            {/* Select mode */}
            <div className="bg-white bg-opacity-50 p-2 items-center justify-center grid gap-4 grid-cols-2 rounded-xl">
              <label class="inline-flex items-center cursor-pointer">
                <input
                  type="checkbox"
                  className="sr-only peer"
                  checked={isChecked}
                  onChange={ModeControl}
                />
                <div class="relative w-11 h-6 bg-gray-200 peer-focus:outline-none  rounded-full peer dark:bg-gray-700 peer-checked:after:translate-x-full rtl:peer-checked:after:-translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:start-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all dark:border-gray-600 peer-checked:bg-green-600"></div>
                <span class="ms-3 text-sm font-medium text-gray-900 dark:text-gray-300">
                  {isChecked ? "Mode Manual" : "Mode Auto"}
                </span>
              </label>
            </div>
            {/* End Select mode */}
            {isChecked ? (
              <div className="p-4 h-5/6 w-full  ">
                <div className=" h-full w-full">
                  <div className="max-w-4xl mx-auto p-6 bg-white border border-gray-300 rounded-3xl shadow-md">
                    <h1 className="text-3xl font-bold mb-6 text-gray-700">
                      Pump Control
                    </h1>
                    {pumps.map((pump, index) => (
                      <div
                        key={index}
                        className={`flex items-center justify-between p-4 mb-4 bg-white border border-gray-300 rounded-md shadow-sm ${
                          pump ? "bg-gray-200" : ""
                        }`}
                      >
                        <span
                          className={`text-lg font-medium ${
                            pump ? "text-blue-600" : "text-gray-600"
                          }`}
                        >
                          Pump {index + 1}: {pump ? "ON" : "OFF"}
                        </span>
                        <button
                          onClick={() => handleClick(index)}
                          className={`px-4 py-2 rounded-md font-medium text-white transition-colors duration-200 ${
                            isLocked
                              ? "bg-gray-500 cursor-not-allowed"
                              : pump
                              ? "bg-red-600 hover:bg-red-700"
                              : "bg-green-600 hover:bg-green-700"
                          }`}
                          disabled={isLocked}
                        >
                          {isLocked
                            ? `Locked: ${formatTime(timeLeft)}`
                            : pump
                            ? "Turn Off"
                            : "Turn On"}
                        </button>
                      </div>
                    ))}
                  </div>
                </div>
              </div>
            ) : (
              <div className="p-4 h-5/6 w-full  ">
                <div className=" h-full w-full">
                  <div class="flex flex-col gap-3">
                    <div className="relative p-4 h-60 mt-2 bg-white border-2 border-blue-500 rounded-2xl items-center justify-center">
                      <div className="ml-5" style={{ maxWidth: "400px" }}>
                        <Bar data={data2} options={optionsBar} />
                      </div>
                    </div>

                    <div className="p-4 relative h-60 mt-2 bg-white border-2 border-blue-500 rounded-2xl ">
                      {/* <div className="mb-4">
                        <h2 className="text-md font-semibold mb-2">
                          Pump Start
                        </h2>
                        <p className="text-gray-600 mb-2">
                          Humidity less than{" "}
                          <span className="font-bold">{startThreshold}%</span>
                        </p>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={startThreshold}
                            onChange={handleStartThresholdChange}
                            className="w-2/3 appearance-none h-3 bg-gray-200 rounded-lg overflow-hidden cursor-pointer"
                            style={{
                              background: `linear-gradient(to right, red, green ${startThreshold}%, #ccc ${startThreshold}%)`,
                            }}
                          />
                          <span className="text-md font-medium">
                            {startThreshold}%
                          </span>
                          <button
                            onClick={handleSaveClick}
                            className="ml-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                          >
                            Save
                          </button>
                        </div>
                      </div>

                      <div>
                        <h2 className="text-md font-semibold mb-2">
                          Pump Stop
                        </h2>
                        <p className="text-gray-600 mb-2">
                          Humidity greater than{" "}
                          <span className="font-bold">{stopThreshold}%</span>
                        </p>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={stopThreshold}
                            onChange={handleStopThresholdChange}
                            className="w-2/3 appearance-none h-3 bg-gray-200 rounded-lg overflow-hidden cursor-pointer"
                            style={{
                              background: `linear-gradient(to right, green, red ${stopThreshold}%, #ccc ${stopThreshold}%)`,
                            }}
                          />
                          <span className="text-md font-medium">
                            {stopThreshold}%
                          </span>
                          <button
                            onClick={handleSaveClick}
                            className="ml-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                          >
                            Save
                          </button>
                        </div>
                      </div> */}
                      <div className="mb-4">
                        <h2 className="text-md font-semibold mb-2">
                          Pump Start
                        </h2>
                        <p className="text-gray-600 mb-2">
                          Humidity less than{" "}
                          <span className="font-bold">{startThreshold}%</span>
                        </p>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={startThreshold}
                            onChange={handleStartThresholdChange}
                            className="w-2/3 appearance-none h-3 bg-gray-200 rounded-lg overflow-hidden cursor-pointer"
                            style={{
                              background: `linear-gradient(to right, red, green ${startThreshold}%, #ccc ${startThreshold}%)`,
                            }}
                          />
                          <span className="text-md font-medium">
                            {startThreshold}%
                          </span>
                          <button
                            onClick={handleSaveClick}
                            className="ml-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                          >
                            Save
                          </button>
                        </div>
                      </div>

                      <div>
                        <h2 className="text-md font-semibold mb-2">
                          Pump Stop
                        </h2>
                        <p className="text-gray-600 mb-2">
                          Humidity greater than{" "}
                          <span className="font-bold">{stopThreshold}%</span>
                        </p>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={stopThreshold}
                            onChange={handleStopThresholdChange}
                            className="w-2/3 appearance-none h-3 bg-gray-200 rounded-lg overflow-hidden cursor-pointer"
                            style={{
                              background: `linear-gradient(to right, green, red ${stopThreshold}%, #ccc ${stopThreshold}%)`,
                            }}
                          />
                          <span className="text-md font-medium">
                            {stopThreshold}%
                          </span>
                          <button
                            onClick={handleSaveClick}
                            className="ml-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                          >
                            Save
                          </button>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            )}
          </div>
          {/* -----------------------End Control Pump---------------------- */}
        </div>
      </div>

      {/* -----------------------Mobile View----------------------- */}
      <div className="sm:hidden h-screen w-screen ">
        <ul className=" p-2 flex justify-center w-full overflow-x-auto ">
          <li class="p-4   list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded ">
            <span
              class="ml-2 underline hover:underline-offset-8  "
              onClick={() => {
                setDisplay("1");
              }}
            >
              Pump View
            </span>
          </li>
          <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-4">
            <span
              class="ml-2 underline hover:underline-offset-8 "
              onClick={() => {
                setDisplay("0");
              }}
            >
              Pump Control
            </span>
          </li>
        </ul>
        {Display === "1" && (
          <div className=" ">
            <h1 className="ml-4 font-bold text-center mb-4 ">PUMP VIEW</h1>
            <div className="flex left-1 gap-3 items-center justify-center">
              <div className=" flex flex-col  items-center h-1/6 w-36 p-2  shadow-xl bg-white text-black rounded-md mr-2 ">
                <div class="flex justify-between text-black">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                    class="icon icon-tabler icons-tabler-outline icon-tabler-engine"
                  >
                    <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                    <path d="M3 10v6" />
                    <path d="M12 5v3" />
                    <path d="M10 5h4" />
                    <path d="M5 13h-2" />
                    <path d="M6 10h2l2 -2h3.382a1 1 0 0 1 .894 .553l1.448 2.894a1 1 0 0 0 .894 .553h1.382v-2h2a1 1 0 0 1 1 1v6a1 1 0 0 1 -1 1h-2v-2h-3v2a1 1 0 0 1 -1 1h-3.465a1 1 0 0 1 -.832 -.445l-1.703 -2.555h-2v-6z" />
                  </svg>
                  <p className="text-left">
                    <b>Motor 1</b>
                  </p>
                </div>

                <div class="ml-2 w-full flex-1">
                  <div class="mt-1 text-base text-gray-600">
                    Status : <span>{valueMotor1 ? "ON" : "Off"}</span>
                  </div>
                  <div class="mt-1 text-base text-gray-600">Time : </div>
                  <div class="mt-1 text-base text-gray-600">Flow : </div>
                </div>
              </div>
              <div className=" flex flex-col  items-center h-1/6 w-36 p-2 shadow-xl bg-white text-black rounded-md mr-2 ">
                <div class="flex justify-between text-black">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                    class="icon icon-tabler icons-tabler-outline icon-tabler-engine"
                  >
                    <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                    <path d="M3 10v6" />
                    <path d="M12 5v3" />
                    <path d="M10 5h4" />
                    <path d="M5 13h-2" />
                    <path d="M6 10h2l2 -2h3.382a1 1 0 0 1 .894 .553l1.448 2.894a1 1 0 0 0 .894 .553h1.382v-2h2a1 1 0 0 1 1 1v6a1 1 0 0 1 -1 1h-2v-2h-3v2a1 1 0 0 1 -1 1h-3.465a1 1 0 0 1 -.832 -.445l-1.703 -2.555h-2v-6z" />
                  </svg>
                  <p className="text-left">
                    <b>Motor 2</b>
                  </p>
                </div>

                <div class="ml-2 w-full flex-1">
                  <div class="mt-1 text-base text-gray-600">
                    Status : <span>{valueMotor2 ? "ON" : "Off"}</span>
                  </div>
                  <div class="mt-1 text-base text-gray-600">Time : </div>
                  <div class="mt-1 text-base text-gray-600">Flow : </div>
                </div>
              </div>
              <div className=" flex flex-col  items-center h-1/6 w-36 p-2 shadow-xl bg-white text-black rounded-md mr-2 ">
                <div class="flex justify-between text-black">
                  <svg
                    xmlns="http://www.w3.org/2000/svg"
                    width="24"
                    height="24"
                    viewBox="0 0 24 24"
                    fill="none"
                    stroke="currentColor"
                    stroke-width="2"
                    stroke-linecap="round"
                    stroke-linejoin="round"
                    class="icon icon-tabler icons-tabler-outline icon-tabler-engine"
                  >
                    <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                    <path d="M3 10v6" />
                    <path d="M12 5v3" />
                    <path d="M10 5h4" />
                    <path d="M5 13h-2" />
                    <path d="M6 10h2l2 -2h3.382a1 1 0 0 1 .894 .553l1.448 2.894a1 1 0 0 0 .894 .553h1.382v-2h2a1 1 0 0 1 1 1v6a1 1 0 0 1 -1 1h-2v-2h-3v2a1 1 0 0 1 -1 1h-3.465a1 1 0 0 1 -.832 -.445l-1.703 -2.555h-2v-6z" />
                  </svg>
                  <p className="text-left">
                    <b>Motor 3 </b>
                  </p>
                </div>

                <div class="ml-2 w-full flex-1">
                  <div class="mt-1 text-base text-gray-600">
                    Status : <span>{valueMotor3 ? "ON" : "Off"}</span>
                  </div>
                  <div class="mt-1 text-base text-gray-600">Time : </div>
                  <div class="mt-1 text-base text-gray-600">Flow : </div>
                </div>
              </div>
            </div>
            <div className=" h-2/3 w-full p-4">
              <div className="bg-white h-full w-full rounded-3xl">
                <ul className=" flex justify-center w-full overflow-x-auto ">
                  <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-2">
                    <span
                      class="ml-2 "
                      onClick={() => {
                        setSelector("motor1");
                      }}
                    >
                      MOTOR 1
                    </span>
                  </li>
                  <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-2">
                    <span
                      class="ml-2 "
                      onClick={() => {
                        setSelector("motor2");
                      }}
                    >
                      MOTOR 2
                    </span>
                  </li>
                  <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-2">
                    <span
                      class="ml-2 "
                      onClick={() => {
                        setSelector("motor3");
                      }}
                    >
                      MOTOR 3
                    </span>
                  </li>
                </ul>
                <div id="chart_Sensor_1" className="p-2  ">
                  <Line
                    data={chartData("#f15bb5", selector)}
                    options={options(selector)}
                  />
                </div>
              </div>
            </div>
          </div>
        )}
        {Display === "0" && (
          <div className=" ">
            <h1 className="ml-4 font-bold text-center mb-4"> Control Pump</h1>
            {/* Select mode */}
            <div className="bg-white bg-opacity-50 p-2 items-center justify-center grid gap-4 grid-cols-2 rounded-xl">
              <label class="inline-flex items-center cursor-pointer">
                <input
                  type="checkbox"
                  className="sr-only peer"
                  checked={isChecked}
                  onChange={ModeControl}
                />
                <div class="relative w-11 h-6 bg-gray-200 peer-focus:outline-none  rounded-full peer dark:bg-gray-700 peer-checked:after:translate-x-full rtl:peer-checked:after:-translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:start-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-5 after:w-5 after:transition-all dark:border-gray-600 peer-checked:bg-green-600"></div>
                <span class="ms-3 text-sm font-medium text-gray-900 dark:text-gray-300">
                  {isChecked ? "Mode Manual" : "Mode Auto"}
                </span>
              </label>
            </div>
            {/* End Select mode */}
            {isChecked ? (
              <div className="p-4 h-5/6 w-full  ">
                <div className=" h-full w-full">
                  <div className="max-w-4xl mx-auto p-6 bg-white border border-gray-300 rounded-3xl shadow-md">
                    <h1 className="text-3xl font-bold mb-6 text-gray-700">
                      Pump Control
                    </h1>
                    {pumps.map((pump, index) => (
                      <div
                        key={index}
                        className={`flex items-center justify-between p-4 mb-4 bg-white border border-gray-300 rounded-md shadow-sm ${
                          pump ? "bg-gray-200" : ""
                        }`}
                      >
                        <span
                          className={`text-lg font-medium ${
                            pump ? "text-blue-600" : "text-gray-600"
                          }`}
                        >
                          Pump {index + 1}: {pump ? "ON" : "OFF"}
                        </span>
                        <button
                          onClick={() => handleClick(index)}
                          className={`px-4 py-2 rounded-md font-medium text-white transition-colors duration-200 ${
                            isLocked
                              ? "bg-gray-500 cursor-not-allowed"
                              : pump
                              ? "bg-red-600 hover:bg-red-700"
                              : "bg-green-600 hover:bg-green-700"
                          }`}
                          disabled={isLocked}
                        >
                          {isLocked
                            ? `Locked: ${formatTime(timeLeft)}`
                            : pump
                            ? "Turn Off"
                            : "Turn On"}
                        </button>
                      </div>
                    ))}
                  </div>
                </div>
              </div>
            ) : (
              <div className="p-4 h-5/6 w-full  ">
                <div className=" h-full w-full">
                  <div class="flex flex-col gap-3">
                    <div className="relative p-4 h-60 mt-2 bg-white border-2 border-blue-500 rounded-2xl items-center justify-center">
                      <div className="ml-5" style={{ maxWidth: "400px" }}>
                        <Bar data={data2} options={optionsBar} />
                      </div>
                    </div>

                    <div className="p-4 relative h-60 mt-2 bg-white border-2 border-blue-500 rounded-2xl ">
                      <div className="mb-4">
                        <h2 className="text-md font-semibold mb-2">
                          Pump Start
                        </h2>
                        <p className="text-gray-600 mb-2">
                          Humidity less than{" "}
                          <span className="font-bold">{startThreshold}%</span>
                        </p>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={startThreshold}
                            onChange={handleStartThresholdChange}
                            className="w-2/3 appearance-none h-3 bg-gray-200 rounded-lg overflow-hidden cursor-pointer"
                            style={{
                              background: `linear-gradient(to right, red, green ${startThreshold}%, #ccc ${startThreshold}%)`,
                            }}
                          />
                          <span className="text-md font-medium">
                            {startThreshold}%
                          </span>
                          <button
                            onClick={handleSaveClick}
                            className="ml-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                          >
                            Save
                          </button>
                        </div>
                      </div>

                      <div>
                        <h2 className="text-md font-semibold mb-2">
                          Pump Stop
                        </h2>
                        <p className="text-gray-600 mb-2">
                          Humidity greater than{" "}
                          <span className="font-bold">{stopThreshold}%</span>
                        </p>
                        <div className="flex items-center gap-2">
                          <input
                            type="range"
                            min="0"
                            max="100"
                            value={stopThreshold}
                            onChange={handleStopThresholdChange}
                            className="w-2/3 appearance-none h-3 bg-gray-200 rounded-lg overflow-hidden cursor-pointer"
                            style={{
                              background: `linear-gradient(to right, green, red ${stopThreshold}%, #ccc ${stopThreshold}%)`,
                            }}
                          />
                          <span className="text-md font-medium">
                            {stopThreshold}%
                          </span>
                          <button
                            onClick={handleSaveClick}
                            className="ml-2 px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                          >
                            Save
                          </button>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default Control;

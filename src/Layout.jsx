import React, { useEffect } from "react";
import { useState } from "react";
import { Dialog } from "@headlessui/react";
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faTree } from "@fortawesome/free-solid-svg-icons";
import { Link, Outlet, useLocation } from "react-router-dom";
import axios from "axios";
import * as Thresh from "./pages/include/DefaultData";

const GetDataTime = 0.5 * 60 *1000;
const Layout = () => {
  const location = useLocation();
  // user information
  const [displayNavigateBar, setDisplay] = useState("0");
  const [fullName, setFullname] = useState("");
  const [Role, setRole] = useState("");
  const [imageSrc, setImageSrc] = useState(null);
  const [isDialogOpen, setIsDialogOpen] = useState(false);
  const [mode, setMode] = useState(false);
  const [PassToCheck, SetPassCheck] = useState("");
  
// project information now
  const [NameProject, setNameProject] = useState([]);
  const [startDay, setstartDay] = useState([]);
  const [Area, setArea] = useState([]);
  const [Quantity, setQuantity] = useState([]);

//create new project information
  const [newNameProject, setnewNameProject] = useState([]);
  const [newstartDay, setnewstartDay] = useState([]);
  const [newArea, setnewArea] = useState([]);
  const [newQuantity, setnewQuantity] = useState([]);
  const [newHarvestDate, setnewHarvestDate] = useState([]);
  const [newStage, setnewnewStage] = useState([]);

// sensor data
const [valueCO2, setValueCO2] = useState(0);
const [valueTemp, setValueTemp] = useState(0);
const [valueHumi, setValueHumi] = useState(0);
const [valueEC, setValueEC] = useState(0);
const [valuePressure, setValuePressure] = useState(0);
const [valueFlowmeters, setValueFlowmeters] = useState(0);
const [valueAlkalinity, setValueAlkalinity] = useState(0);
const [valueMotor1, setValueMotor1] = useState(false);
const [valueMotor2, setValueMotor2] = useState(false);
const [valueMotor3, setValueMotor3] = useState(false);
const [valueFullTank, setValueFullTank] = useState(false);
const [valueEmptyTank, setValueEmptyTank] = useState(false);
const [data1, setData] = useState([]);
const [data2, setData2] = useState([]);



// get local inf
  localStorage.setItem("role", Role);
  localStorage.setItem("full_name", fullName);

// open Dialog to change information project
  const openDialog = (mode) => {
    setMode(mode);
    setIsDialogOpen(true);
  };
  // Lấy token
  const token = localStorage.getItem("token");
  const access_token = "Bearer " + token;


  // logout account
  const goOut = async () => {
    localStorage.clear();
    window.location.href = "http://34.87.151.244:5173/";
  };

  //set status navigate bar
  const ToggleListSettings = async () => {
    setDisplay((prevDisplay) => (prevDisplay === "1" ? "0" : "1"));
  };

  // get user information
  const getInf = async () => {
    const response = await axios.get("http://34.87.151.244:1506/users/me", {
      headers: {
        Authorization: access_token,
        accept: "application/json",
        "Content-Type": "application/x-www-form-urlencoded",
      },
    });

    setRole(response.data["role"]);
    setFullname(response.data["full_name"]);
  };

  // get project information
  const getIn4 = async () => {
    const respone = await axios.get("http://34.87.151.244:1506/crop", {
      headers: {
        Authorization: access_token,
      },
    });

    const dt = respone.data;
    setnewArea(dt.area);
    setnewNameProject(dt.project);
    setnewQuantity(dt.quantity);
    setnewstartDay(dt.startdate);
    setnewnewStage(dt.stage);
    setnewHarvestDate(dt.harvestdate);
  };

  // thay đổi avartar theo role,username
  const imagePath = new URL(
    `/src/assets/user/${localStorage.getItem("username")}.jpg`,
    import.meta.url
  ).href;

// get data sensor 
async function loadData() {

  console.log("start get data");
  const response = await axios.get("http://34.87.151.244:1506/api/data/0", {
    headers: {
      Authorization: access_token,
      accept: "application/json",
      "Content-Type": "application/x-www-form-urlencoded",
    },
  });
  const dt1 = response.data;
  // console.log("TAO NE dt1 ",typeof dt1);

  setData(dt1);
  localStorage.setItem("dataSensor",  JSON.stringify(dt1));


  const responseMotor = await axios.get(
    "http://34.87.151.244:1506/api/motor/0",
    {
      headers: {
        Authorization: access_token,
        accept: "application/json",
        "Content-Type": "application/x-www-form-urlencoded",
      },
    }
  );
  const dt2 = responseMotor.data;
  setData2(dt2);
  localStorage.setItem("dataMotor",JSON.stringify(dt2));
 
  console.log("get data successful");

}

  const handleImageClick = () => {
    document.getElementById("fileInput").click();
  };
  const handleImageChange = (event) => {
    const file = event.target.files[0];
    if (file) {
      const reader = new FileReader();
      reader.onloadend = () => {
        setImageSrc(reader.result);
        uploadImage(file);
      };
      reader.readAsDataURL(file);
    }
  };

  // upload avatar user
  const uploadImage = async (file) => {
    const formData = new FormData();
    formData.append("file", file);
    formData.append("username", localStorage.getItem("username"));

    try {
      const response = await fetch("http://34.87.151.244:1506/upload-img", {
        method: "POST",
        body: formData,
      });

      if (response.ok) {
        const data = await response.json();
        console.log("Image uploaded successfully:", data);
      } else {
        console.error("Image upload failed");
      }
    } catch (error) {
      console.error("Error uploading image:", error);
    }
  };
  const closeDialog = async () => {
    setIsDialogOpen(false);
  };
  const sendChange = async () => {
    const response = axios.post(
      "http://34.87.151.244:1506/hv_in4",
      {
        info: {
          project: NameProject,
          startdate: startDay,
          quantity: Quantity,
          area: Area,
        },
        usr: {
          masterusr: localStorage.getItem("username"),
          masterpwd: PassToCheck,
        },
      },
      {
        headers: {
          Authorization: access_token,
          accept: "application/json",
          "Content-Type": "application/json",
        },
      }
    );
    setIsDialogOpen(false);

    getIn4();
  };

  //get inf once time
  useEffect(() => {
    getInf();
    getIn4();
  
  },[]); // Chỉ chạy một lần khi component mount

// spamdata and check warning
setInterval(()=>{
 
  
  loadData();
 }, GetDataTime);


  return (
    <>
      {location.pathname != "/" && (
        <div className="flex">
          <Dialog
            open={isDialogOpen}
            onClose={closeDialog}
            className="fixed inset-0 z-10 overflow-y-auto"
          >
            {mode ? (
              <div className="flex items-center bg-opacity-75 bg-black justify-center min-h-screen px-4">
                <div className="relative bg-white rounded-lg max-w-sm mx-auto p-6">
                  <h1>Change Information</h1>
                  <div class="relative">
                    <input
                      type="text"
                      id="floating_outlined"
                      class="block px-2.5 pb-2.5 pt-4 w-full text-sm text-gray-900 bg-transparent rounded-lg border-2 border-gray-500 appearance-none dark:text-white dark:border-gray-600 dark:focus:border-blue-500 focus:outline-none focus:ring-0 focus:border-blue-600 peer"
                      placeholder=" "
                      onChange={(e) => setNameProject(e.target.value)}
                    />
                    <label
                      for="floating_outlined"
                      class="absolute text-sm text-gray-500 dark:text-gray-400 duration-300 transform -translate-y-4 scale-75 top-2 z-10 origin-[0] bg-white dark:bg-gray-900 px-2 peer-focus:px-2 peer-focus:text-blue-600 peer-focus:dark:text-blue-500 peer-placeholder-shown:scale-100 peer-placeholder-shown:-translate-y-1/2 peer-placeholder-shown:top-1/2 peer-focus:top-2 peer-focus:scale-75 peer-focus:-translate-y-4 rtl:peer-focus:translate-x-1/4 rtl:peer-focus:left-auto start-1"
                    >
                      Project Name
                    </label>
                  </div>
                  <div class="relative mt-3">
                    <input
                      type="date"
                      id="floating_outlined"
                      class="block px-2.5 pb-2.5 pt-4 w-full text-sm text-gray-900 bg-transparent rounded-lg border-2 border-gray-500 appearance-none dark:text-white dark:border-gray-600 dark:focus:border-blue-500 focus:outline-none focus:ring-0 focus:border-blue-600 peer"
                      placeholder="YYYY-MM-DD"
                      onChange={(e) => setstartDay(e.target.value)}
                    />
                    <label
                      for="floating_outlined"
                      class="absolute text-sm   w-32 text-gray-500 dark:text-gray-400 duration-300 transform -translate-y-4 scale-75 top-2 z-10 origin-[0] bg-white dark:bg-gray-900 px-2 peer-focus:px-2 peer-focus:text-blue-600 peer-focus:dark:text-blue-500 peer-placeholder-shown:scale-100 peer-placeholder-shown:-translate-y-1/2 peer-placeholder-shown:top-1/2 peer-focus:top-2 peer-focus:scale-75 peer-focus:-translate-y-4 rtl:peer-focus:translate-x-1/4 rtl:peer-focus:left-auto start-1 peer-focus:w-fit"
                    >
                      Start Day
                    </label>
                  </div>
                  <div class="relative mt-3">
                    <input
                      type="text"
                      id="floating_outlined"
                      class="block px-2.5 pb-2.5 pt-4 w-full text-sm text-gray-900 bg-transparent rounded-lg border-2 border-gray-500 appearance-none dark:text-white dark:border-gray-600 dark:focus:border-blue-500 focus:outline-none focus:ring-0 focus:border-blue-600 peer"
                      placeholder=" "
                      onChange={(e) => setQuantity(e.target.value)}
                    />
                    <label
                      for="floating_outlined"
                      class="absolute text-sm text-gray-500 dark:text-gray-400 duration-300 transform -translate-y-4 scale-75 top-2 z-10 origin-[0] bg-white dark:bg-gray-900 px-2 peer-focus:px-2 peer-focus:text-blue-600 peer-focus:dark:text-blue-500 peer-placeholder-shown:scale-100 peer-placeholder-shown:-translate-y-1/2 peer-placeholder-shown:top-1/2 peer-focus:top-2 peer-focus:scale-75 peer-focus:-translate-y-4 rtl:peer-focus:translate-x-1/4 rtl:peer-focus:left-auto start-1"
                    >
                      Quantity
                    </label>
                  </div>
                  <div class="relative mt-3">
                    <input
                      type="text"
                      id="floating_outlined"
                      class="block px-2.5 pb-2.5 pt-4 w-full text-sm text-gray-900 bg-transparent rounded-lg border-2 border-gray-500 appearance-none dark:text-white dark:border-gray-600 dark:focus:border-blue-500 focus:outline-none focus:ring-0 focus:border-blue-600 peer"
                      placeholder=" "
                      onChange={(e) => setArea(e.target.value)}
                    />
                    <label
                      for="floating_outlined"
                      class="absolute text-sm text-gray-500 dark:text-gray-400 duration-300 transform -translate-y-4 scale-75 top-2 z-10 origin-[0] bg-white dark:bg-gray-900 px-2 peer-focus:px-2 peer-focus:text-blue-600 peer-focus:dark:text-blue-500 peer-placeholder-shown:scale-100 peer-placeholder-shown:-translate-y-1/2 peer-placeholder-shown:top-1/2 peer-focus:top-2 peer-focus:scale-75 peer-focus:-translate-y-4 rtl:peer-focus:translate-x-1/4 rtl:peer-focus:left-auto start-1"
                    >
                      Area
                    </label>
                  </div>
                  <div className="mt-4 flex justify-end">
                    <button
                      onClick={closeDialog}
                      className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                    >
                      Cancel
                    </button>

                    <button
                      onClick={() => openDialog(false)}
                      className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                    >
                      Submit
                    </button>
                  </div>
                </div>
              </div>
            ) : (
              <div className="flex items-center bg-opacity-75 bg-black justify-center min-h-screen px-4">
                <div className="relative bg-white rounded-lg max-w-sm mx-auto p-6">
                  <div className="text-lg font-bold text-gray-900">
                    Enter Password
                  </div>
                  <div className="mt-2 text-sm text-gray-500">
                    Please enter a password to proceed.
                  </div>

                  <input
                    type="password"
                    id="password"
                    name="password"
                    className="mt-4 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:border-green-500 focus:ring focus:ring-green-500 focus:ring-opacity-50"
                    onChange={(e) => SetPassCheck(e.target.value)}
                  />

                  <div className="mt-4 flex justify-end">
                    <button
                      onClick={closeDialog}
                      className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                    >
                      Cancel
                    </button>
                    <button
                      onClick={sendChange}
                      className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                    >
                      Submit
                    </button>
                  </div>
                </div>
              </div>
            )}
          </Dialog>
          {/* PC View */}
          <div className="hidden sm:block h-screen w-screen max-h-fit max-ww-fit gradient-background">
            <div className="p-10 h-screen w-screen max-h-fit max-ww-fit gradient-background ">
              <div className=" flex  bg-white bg-opacity-50 h-full rounded-l-3xl rounded-r-3xl">
                {/* ------------------------------List Box-----------------------------*/}

                <div className="p-0 w-1/5 hidden md:block">
                  <div className="p-4 w-46 rounded-r-xl rounded-l-xl bg-white Green_screen">
                    <div className="flex items-center h-3/5 w-full">
                      <img
                        className="block mx-auto h-16 w-16 rounded-full sm:mx-0 sm:flex-shrink-0"
                        src={imagePath}
                        alt={`${localStorage.getItem("role")}`}
                        onClick={handleImageClick}
                      />
                      <input
                        id="fileInput"
                        type="file"
                        style={{ display: "none" }}
                        onChange={handleImageChange}
                      />
                      <div style={{ marginLeft: 10 }}>
                        <p>
                          <strong>{localStorage.getItem("full_name")}</strong>
                        </p>
                        <span>{Role}</span>
                      </div>
                    </div>
                    <br />
                    <ul className="fmt-6 leading-10 flex flex-col space-y-4">
                      <a
                        className="inline-flex items-center w-full text-lg text-green-500 font-bold transition-colors duration-150 cursor-pointer hover:text-teal-600"
                        href=""
                      >
                        <svg
                          xmlns="http://www.w3.org/2000/svg"
                          width="34"
                          height="34"
                          viewBox="0 0 24 24"
                          fill="none"
                          stroke="currentColor"
                          strokeWidth="2"
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          className="icon icon-tabler icons-tabler-outline icon-tabler-home"
                        >
                          <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                          <path d="M5 12l-2 0l9 -9l9 9l-2 0" />
                          <path d="M5 12v7a2 2 0 0 0 2 2h10a2 2 0 0 0 2 -2v-7" />
                          <path d="M9 21v-6a2 2 0 0 1 2 -2h2a2 2 0 0 1 2 2v6" />
                        </svg>

                        <Link to="/home">
                          <span
                            className="ml-4"
                            onClick={() => {
                              setDisplay("0");
                            }}
                          >
                            Dashboard
                          </span>
                        </Link>
                      </a>
                      {Role === "admin" && (
                        <a
                          className="inline-flex items-center w-full text-lg text-green-500 font-bold transition-colors duration-150 cursor-pointer hover:text-teal-600"
                          href=""
                        >
                          <svg
                            xmlns="http://www.w3.org/2000/svg"
                            width="34"
                            height="34"
                            viewBox="0 0 24 24"
                            fill="none"
                            stroke="currentColor"
                            strokeWidth="2"
                            strokeLinecap="round"
                            strokeLinejoin="round"
                            className="icon icon-tabler icons-tabler-outline icon-tabler-users"
                          >
                            <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                            <path d="M9 7m-4 0a4 4 0 1 0 8 0a4 4 0 1 0 -8 0" />
                            <path d="M3 21v-2a4 4 0 0 1 4 -4h4a4 4 0 0 1 4 4v2" />
                            <path d="M16 3.13a4 4 0 0 1 0 7.75" />
                            <path d="M21 21v-2a4 4 0 0 0 -3 -3.85" />
                          </svg>

                          <Link to="/user-management">
                            <span
                              className="ml-4"
                              onClick={() => {
                                setDisplay("0");
                              }}
                            >
                              Users
                            </span>
                          </Link>
                        </a>
                      )}
                      <a
                        className="inline-flex items-center w-full text-lg text-green-500 font-bold transition-colors duration-150 cursor-pointer hover:text-teal-600"
                        href=""
                      >
                        <svg
                          xmlns="http://www.w3.org/2000/svg"
                          width="34"
                          height="34"
                          viewBox="0 0 24 24"
                          fill="none"
                          stroke="currentColor"
                          stroke-width="2"
                          stroke-linecap="round"
                          stroke-linejoin="round"
                          class="icon icon-tabler icons-tabler-outline icon-tabler-adjustments-horizontal"
                        >
                          <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                          <path d="M14 6m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                          <path d="M4 6l8 0" />
                          <path d="M16 6l4 0" />
                          <path d="M8 12m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                          <path d="M4 12l2 0" />
                          <path d="M10 12l10 0" />
                          <path d="M17 18m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                          <path d="M4 18l11 0" />
                          <path d="M19 18l1 0" />
                        </svg>
                        <Link to="/control">
                          <span
                            className="ml-4"
                            onClick={() => {
                              setDisplay("0");
                            }}
                          >
                            Control Panel
                          </span>
                        </Link>
                      </a>
                      <a
                        className="inline-flex items-center w-full text-lg text-green-500 font-bold transition-colors duration-150 cursor-pointer hover:text-teal-600"
                        href=""
                      >
                        <svg
                          xmlns="http://www.w3.org/2000/svg"
                          width="34"
                          height="34"
                          viewBox="0 0 24 24"
                          fill="none"
                          stroke="currentColor"
                          stroke-width="2"
                          stroke-linecap="round"
                          stroke-linejoin="round"
                          class="icon icon-tabler icons-tabler-outline icon-tabler-file-description"
                        >
                          <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                          <path d="M14 3v4a1 1 0 0 0 1 1h4" />
                          <path d="M17 21h-10a2 2 0 0 1 -2 -2v-14a2 2 0 0 1 2 -2h7l5 5v11a2 2 0 0 1 -2 2z" />
                          <path d="M9 17h6" />
                          <path d="M9 13h6" />
                        </svg>

                        <Link to="/Data-Analysis">
                          <span
                            className="ml-4"
                            onClick={() => {
                              setDisplay("0");
                            }}
                          >
                            History
                          </span>
                        </Link>
                      </a>
                      <a
                        className="ml-1 inline-flex items-center w-full text-lg text-green-500 font-bold transition-colors duration-150 cursor-pointer hover:text-teal-600"
                        href=""
                      >
                        <svg
                          xmlns="http://www.w3.org/2000/svg"
                          width="34"
                          height="34"
                          viewBox="0 0 24 24"
                          fill="none"
                          stroke="currentColor"
                          stroke-width="2"
                          stroke-linecap="round"
                          stroke-linejoin="round"
                          class="icon icon-tabler icons-tabler-outline icon-tabler-logout"
                        >
                          <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                          <path d="M14 8v-2a2 2 0 0 0 -2 -2h-7a2 2 0 0 0 -2 2v12a2 2 0 0 0 2 2h7a2 2 0 0 0 2 -2v-2" />
                          <path d="M9 12h12l-3 -3" />
                          <path d="M18 15l3 -3" />
                        </svg>
                        <span className="ml-4" onClick={() => goOut()}>
                          Logout
                        </span>
                      </a>
                    </ul>
                  </div>
                  <div className="p-2 w-46 rounded-r-xl rounded-l-xl bg-white Green_screen">
                    <p className="ml-4 font-bold text-center mb-4">
                      INFORMATION
                    </p>

                    <table className="ml-4 w-4/5">
                      <tbody>
                        <tr>
                          <th className="text-left">Project:</th>
                          <td className="text-right">{newNameProject}</td>
                        </tr>
                        <tr>
                          <th className="text-left">Start Day:</th>
                          <td className="text-right">{newstartDay}</td>
                        </tr>
                        <tr>
                          <th className="text-left">Harvest Day:</th>
                          <td className="text-right">{newHarvestDate}</td>
                        </tr>
                        <tr>
                          <th className="text-left">Stage:</th>
                          <td className="text-right">{newStage}</td>
                        </tr>
                        <tr>
                          <th className="text-left">Quantity:</th>
                          <td className="text-right">
                            {newQuantity} <FontAwesomeIcon icon={faTree} />
                          </td>
                        </tr>
                        <tr>
                          <th className="text-left">Area:</th>
                          <td className="text-right">
                            {newArea} m<sup>2</sup>
                          </td>
                        </tr>
                      </tbody>
                    </table>

                    <div className="flex justify-center mt-6">
                      <button
                        value="create"
                        onClick={() => openDialog(true)}
                        className="px-4 py-2 button-create-user bg-green-500 transition-colors duration-150 cursor-pointer hover:bg-green-900 text-cyan-50 rounded-full font-bold"
                      >
                        Change Project
                      </button>
                    </div>
                  </div>
                </div>
                {/* -------------------------- End List Box----------------------------- */}
                <div className="p-4 w-4/5">
                  <Outlet />
                </div>
              </div>
            </div>
          </div>
          {/* Mobile View */}
          <div className="sm:hidden h-screen w-screen bg-green-100 bg-opacity-50">
            {/* Inf box*/}
            <div className="h-20 sticky bg-[#059212] top-0">
              <div className="">
                <div className="flex  ">
                  {/* Img */}
                  <div className=" m-3 w-1/5">
                    <img
                      className="h-16 w-16 rounded-full"
                      src={imagePath}
                      alt={`${localStorage.getItem("role")}`}
                      onClick={handleImageClick}
                    />
                    <input
                      id="fileInput"
                      type="file"
                      style={{ display: "none" }}
                      onChange={handleImageChange}
                    />
                  </div>
                  {/* End Img */}

                  {/* User Inf*/}
                  <div className=" mt-4 text-white w-3/5">
                    <p>
                      <strong>{localStorage.getItem("full_name")}</strong>
                    </p>
                    <span>{Role}</span>
                  </div>
                  {/*End User Inf*/}

                  {/* Icon */}
                  <div className="ml-5 mt-5 text-white w-1/5 ">
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      stroke-width="2"
                      stroke-linecap="round"
                      stroke-linejoin="round"
                      class="icon icon-tabler icons-tabler-outline icon-tabler-align-right"
                      onClick={() => ToggleListSettings()}
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M4 6l16 0" />
                      <path d="M10 12l10 0" />
                      <path d="M6 18l14 0" />
                    </svg>
                  </div>
                  {/*End Icon */}
                </div>
              </div>
            </div>
            {/* End Inf box*/}
            {/* List Box */}
            {/* {displayNavigateBar === "1" && (
              <aside className="p-4 fixed inset-y-0 z-20 right-0 flex-shrink-0 w-3/5 mt-20 overflow-y-auto  bg-[#0A6847]">
                <ul className="fmt-6 leading-10 flex flex-col space-y-4">
                  <a
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    href=""
                  >
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
                      class="icon icon-tabler icons-tabler-outline icon-tabler-home"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M5 12l-2 0l9 -9l9 9l-2 0" />
                      <path d="M5 12v7a2 2 0 0 0 2 2h10a2 2 0 0 0 2 -2v-7" />
                      <path d="M9 21v-6a2 2 0 0 1 2 -2h2a2 2 0 0 1 2 2v6" />
                    </svg>
                    <Link to="/home">
                      <span
                        className="ml-4"
                        onClick={() => {
                          setDisplay("0");
                        }}
                      >
                        Dashboard
                      </span>
                    </Link>
                  </a>
                  {Role === "admin" && (
                    <a
                      className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                      href=""
                    >
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        width="34"
                        height="34"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        className="icon icon-tabler icons-tabler-outline icon-tabler-users"
                      >
                        <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                        <path d="M9 7m-4 0a4 4 0 1 0 8 0a4 4 0 1 0 -8 0" />
                        <path d="M3 21v-2a4 4 0 0 1 4 -4h4a4 4 0 0 1 4 4v2" />
                        <path d="M16 3.13a4 4 0 0 1 0 7.75" />
                        <path d="M21 21v-2a4 4 0 0 0 -3 -3.85" />
                      </svg>
                      <Link to="/user-management">
                        <span
                          className="ml-4"
                          onClick={() => {
                            setDisplay("0");
                          }}
                        >
                          Users
                        </span>
                      </Link>
                    </a>
                  )}
                  <a
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    href=""
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      className="icon icon-tabler icons-tabler-outline icon-tabler-adjustments-horizontal"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M14 6m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                      <path d="M4 6l8 0" />
                      <path d="M16 6l4 0" />
                      <path d="M8 12m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                      <path d="M4 12l2 0" />
                      <path d="M10 12l10 0" />
                      <path d="M17 18m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                      <path d="M4 18l11 0" />
                      <path d="M19 18l1 0" />
                    </svg>
                    <Link to="/control">
                      <span
                        className="ml-4"
                        onClick={() => {
                          setDisplay("0");
                        }}
                      >
                        Control Panel
                      </span>
                    </Link>
                  </a>
                  <a
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    href=""
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      className="icon icon-tabler icons-tabler-outline icon-tabler-file-description"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M14 3v4a1 1 0 0 0 1 1h4" />
                      <path d="M17 21h-10a2 2 0 0 1 -2 -2v-14a2 2 0 0 1 2 -2h7l5 5v11a2 2 0 0 1 -2 2z" />
                      <path d="M9 17h6" />
                      <path d="M9 13h6" />
                    </svg>
                    <Link to="/Data-Analysis">
                      <span
                        className="ml-4"
                        onClick={() => {
                          setDisplay("0");
                        }}
                      >
                        History
                      </span>
                    </Link>
                  </a>
                  <a
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    href=""
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      stroke-width="2"
                      stroke-linecap="round"
                      stroke-linejoin="round"
                      class="icon icon-tabler icons-tabler-outline icon-tabler-logout"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M14 8v-2a2 2 0 0 0 -2 -2h-7a2 2 0 0 0 -2 2v12a2 2 0 0 0 2 2h7a2 2 0 0 0 2 -2v-2" />
                      <path d="M9 12h12l-3 -3" />
                      <path d="M18 15l3 -3" />
                    </svg>
                    <span className="ml-4" onClick={() => goOut()}>
                      Logout
                    </span>
                  </a>
                </ul>
              </aside>
            )} */}
            {displayNavigateBar === "1" && (
              <aside className="p-4 fixed inset-y-0 z-20 right-0 flex-shrink-0 w-3/5 mt-20 overflow-y-auto  bg-[#0A6847]">
                <ul className="fmt-6 leading-10 flex flex-col space-y-4">
                  <Link
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    to="/home"
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="24"
                      height="24"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      className="icon icon-tabler icons-tabler-outline icon-tabler-home"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M5 12l-2 0l9 -9l9 9l-2 0" />
                      <path d="M5 12v7a2 2 0 0 0 2 2h10a2 2 0 0 0 2 -2v-7" />
                      <path d="M9 21v-6a2 2 0 0 1 2 -2h2a2 2 0 0 1 2 2v6" />
                    </svg>
                    <span
                      className="ml-4"
                      onClick={() => {
                        setDisplay("0");
                      }}
                    >
                      Dashboard
                    </span>
                  </Link>
                  {Role === "admin" && (
                    <Link
                      className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                      to="/user-management"
                    >
                      <svg
                        xmlns="http://www.w3.org/2000/svg"
                        width="34"
                        height="34"
                        viewBox="0 0 24 24"
                        fill="none"
                        stroke="currentColor"
                        strokeWidth="2"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        className="icon icon-tabler icons-tabler-outline icon-tabler-users"
                      >
                        <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                        <path d="M9 7m-4 0a4 4 0 1 0 8 0a4 4 0 1 0 -8 0" />
                        <path d="M3 21v-2a4 4 0 0 1 4 -4h4a4 4 0 0 1 4 4v2" />
                        <path d="M16 3.13a4 4 0 0 1 0 7.75" />
                        <path d="M21 21v-2a4 4 0 0 0 -3 -3.85" />
                      </svg>
                      <span
                        className="ml-4"
                        onClick={() => {
                          setDisplay("0");
                        }}
                      >
                        Users
                      </span>
                    </Link>
                  )}
                  <Link
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    to="/control"
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      className="icon icon-tabler icons-tabler-outline icon-tabler-adjustments-horizontal"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M14 6m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                      <path d="M4 6l8 0" />
                      <path d="M16 6l4 0" />
                      <path d="M8 12m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                      <path d="M4 12l2 0" />
                      <path d="M10 12l10 0" />
                      <path d="M17 18m-2 0a2 2 0 1 0 4 0a2 2 0 1 0 -4 0" />
                      <path d="M4 18l11 0" />
                      <path d="M19 18l1 0" />
                    </svg>
                    <span
                      className="ml-4"
                      onClick={() => {
                        setDisplay("0");
                      }}
                    >
                      Control Panel
                    </span>
                  </Link>
                  <Link
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    to="/Data-Analysis"
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      className="icon icon-tabler icons-tabler-outline icon-tabler-file-description"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M14 3v4a1 1 0 0 0 1 1h4" />
                      <path d="M17 21h-10a2 2 0 0 1 -2 -2v-14a2 2 0 0 1 2 -2h7l5 5v11a2 2 0 0 1 -2 2z" />
                      <path d="M9 17h6" />
                      <path d="M9 13h6" />
                    </svg>
                    <span
                      className="ml-4"
                      onClick={() => {
                        setDisplay("0");
                      }}
                    >
                      History
                    </span>
                  </Link>
                  <a
                    className="inline-flex items-center w-full text-lg text-white font-bold transition-colors duration-150 cursor-pointer hover:text-green-500"
                    onClick={goOut}
                  >
                    <svg
                      xmlns="http://www.w3.org/2000/svg"
                      width="34"
                      height="34"
                      viewBox="0 0 24 24"
                      fill="none"
                      stroke="currentColor"
                      strokeWidth="2"
                      strokeLinecap="round"
                      strokeLinejoin="round"
                      className="icon icon-tabler icons-tabler-outline icon-tabler-logout"
                    >
                      <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                      <path d="M14 8v-2a2 2 0 0 0 -2 -2h-7a2 2 0 0 0 -2 2v12a2 2 0 0 0 2 2h7a2 2 0 0 0 2 -2v-2" />
                      <path d="M9 12h12l-3 -3" />
                      <path d="M18 15l3 -3" />
                    </svg>
                    <span className="ml-4">Logout</span>
                  </a>
                </ul>
              </aside>
            )}

            {/* End List Box */}
            <div className="flex  h-full w-screen flex-col md:flex-row ">
              {/* Outletx*/}
              <div className=" w-full md:w-4/5  bg-green-100 bg-opacity-50">
                <Outlet />
              </div>

              {/* End Outletx*/}
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default Layout;

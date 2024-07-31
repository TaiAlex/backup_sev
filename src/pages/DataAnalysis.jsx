import React from "react";
import { useEffect, useState } from "react";
// import axios from "axios";
import axios from "./checkToken";

import { Button } from "@material-tailwind/react";

const DataAnalysis = () => {
  const [data1, setData] = useState([]);
  const [DataList, setDataList] = useState("Sensor");

  // Lấy token
  const token = localStorage.getItem("token");
  const access_token = "Bearer " + token;
  const reversedData1 = data1.reverse();
  // lấy trạng thái Motor để hiển thị

  const ImgUsr = (usr) => {
    const a = new URL(`/src/assets/user/${usr}.jpg`, import.meta.url).href;
    if (a === "http://34.87.151.244:5173/src/pages/undefined") {
      return new URL(`/src/assets/user/user.jpg`, import.meta.url).href;
    }
    return a;
  };
  // DATA SENSOR
  useEffect(() => {
    async function loadData() {
      console.log("get history");
      const response = await axios.get(
        "http://34.87.151.244:1506/history/" + `${DataList}`,
        {
          headers: {
            Authorization: access_token,
            accept: "application/json",
            "Content-Type": "application/x-www-form-urlencoded",
          },
        }
      );
      const dt1 = response.data;
      console.log("Data get ", dt1);
      setData(dt1);
    }
    loadData();
  }, [DataList]);

  //Sent Mail
  const SentMail = async () => {
    const username = localStorage.getItem("username");
    const password = localStorage.getItem("password");
    const Account = new URLSearchParams({
      username,
      password,
    }).toString();
    const address_toSend = `http://34.87.151.244:1506/send-file/${DataList}?${Account}`;
    console.log("address_toSend", address_toSend);
    try {
      await fetch(address_toSend, {
        method: "POST", // or 'PUT'
        headers: {
          Authorization: access_token,
          accept: "application/json",
          "Content-Type": "application/x-www-form-urlencoded",
        },
      });
    } catch (error) {
      console.error("Error:", error);
    }
  };
  return (
    <div className="flex-col h-full w-full  ">
      {/* PC View */}
      <div className=" p-2 hidden sm:block w-full h-full">
        <div className="flex-col  h-full w-full data-table-container">
          <div className="h-16 m-3  flex  items-center">
            <div className=" w-2/5">
              <h1 className="text-5xl font-bold">history</h1>
            </div>
            {/* <div className=" w-3/5 flex">
              <label className="inline-flex items-center w-2/3 text-sm font-semibold ">
                <select
                  className="text-gray-500 mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1select_box"
                  name="data-list"
                  id="data-list"
                  onChange={(e) => setDataList(e.target.value)}
                >
                  <option value="admin">Choice data </option>
                  <option value="Sensor">Sensor</option>
                  <option value="user">User</option>
                  <option value="control-History">Control History</option>
                </select>
              </label>
              <div>hi</div>
              <div className="w-1/3">
                <Button
                  className="bg-white text-black float-right text-sm"
                  onClick={() => SentMail()}
                >
                  EXPORT FILE
                </Button>
              </div>
            </div> */}
            <div className="w-11/12 flex">
              <label className="inline-flex items-center w-2/3 text-sm font-semibold">
                <select
                  className="text-gray-500 mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                  name="data-list"
                  id="data-list"
                  onChange={(e) => setDataList(e.target.value)}
                >
                  <option value="admin">Choice data</option>
                  <option value="Sensor">Sensor</option>
                  <option value="user">User</option>
                  <option value="control-History">Control History</option>
                </select>
              </label>
              <div className="w-2/3 flex items-center justify-between px-2">
                <input
                  type="date"
                  className="mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block rounded-md sm:text-sm focus:ring-1"
                  name="end-date"
                  id="end-date"
                  onChange={(e) => setEndDate(e.target.value)}
                />
              </div>
              <div className="w-1/3">
                <Button
                  className="bg-white text-black float-right text-sm"
                  onClick={() => SentMail()}
                >
                  EXPORT FILE
                </Button>
              </div>
            </div>
          </div>
          {DataList === "Sensor" && (
            <div className="relative h-4/5 overflow-x-auto rounded-3xl mt-3">
              <table class="w-full h-46 text-sm text-left rtl:text-right text-gray-500 dark:text-gray-400">
                <thead class="text-xs text-gray-900 uppercase dark:text-gray-400 bg-lime-300 sticky top-0 ">
                  <tr>
                    <th scope="col" class="px-6 py-3 text-center">
                      Device
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Datetime
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Time
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      CO2
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Temp
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Humi
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      EC
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Pressure
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Flowmeters
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Motor
                    </th>
                  </tr>
                </thead>
                <tbody className="">
                  {reversedData1.map((item) => (
                    <tr class="bg-white dark:bg-gray-800" key={item._id}>
                      <td class="px-6 py-4 text-center">{item.device_name}</td>
                      <td class="px-6 py-4 text-center">{item.date}</td>
                      <td class="px-6 py-4 text-center">{item.time}</td>
                      <td class="px-6 py-4 text-center">{item.CO2}</td>
                      <td class="px-6 py-4 text-center">{item.Temp}</td>
                      <td class="px-6 py-4 text-center">{item.Humi}</td>
                      <td class="px-6 py-4 text-center">{item.EC}</td>
                      <td class="px-6 py-4 text-center">{item.Pressure}</td>
                      <td class="px-6 py-4 text-center">{item.Flowmeters}</td>
                      <td class="px-6 py-4 text-center">
                        {item.Motor ? "On" : "Off"}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          {DataList === "user" && (
            <div className="relative h-4/5  overflow-x-auto  rounded-3xl mt-3">
              <table class="w-full text-sm text-left rtl:text-right text-gray-500 rounded-3xl dark:text-gray-400">
                <thead class="text-xs text-gray-900 uppercase dark:text-gray-400 bg-lime-300 sticky top-0">
                  <tr>
                    <th scope="col" class="px-6 py-3 text-center">
                      Name
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      User Name
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Phone Number
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Email
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Role
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Login at
                    </th>
                  </tr>
                </thead>
                <tbody className="rounded-b-2xl ">
                  {reversedData1.map((item) => (
                    <tr class="bg-white dark:bg-gray-800" key={item._id}>
                      <td class="px-6 py-4 text-center">{item.full_name}</td>
                      <td class="px-6 py-4 text-center">{item.username}</td>

                      <td class="px-6 py-4 text-center">{item.phone}</td>
                      <td class="px-6 py-4 text-center">{item.email}</td>
                      <td class="px-6 py-4 text-center">{item.role}</td>
                      <td class="px-6 py-4 text-center">{item.lastlogin}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          {DataList === "Control-History" && (
            <div className="relative h-4/5 bg-white overflow-x-auto  rounded-3xl mt-3"></div>
          )}
        </div>
      </div>
      {/* Mobile View */}
      <div className="sm:hidden h-screen w-screen">
        <div className="flex-col  h-full w-full data-table-container">
          <div className="h-16 m-3  flex  items-center">
            <div className=" w-full flex">
              <label className="inline-flex items-center w-7/12 h-10 text-sm font-semibold ">
                <select
                  className="text-gray-500 mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1select_box"
                  name="data-list"
                  id="data-list"
                  onChange={(e) => setDataList(e.target.value)}
                >
                  <option value="admin">Choice data </option>
                  <option value="Sensor">Sensor</option>
                  <option value="user">User</option>
                  <option value="control-History">Control History</option>
                </select>
              </label>
              <div className="w-fit  ml-2 ">
                <Button
                  className="bg-lime-600 text-white float-right text-sm"
                  onClick={() => SentMail()}
                >
                  EXPORT FILE
                </Button>
              </div>
            </div>
          </div>
          {DataList === "Sensor" && (
            <div className="relative h-4/5 overflow-x-auto rounded-3xl mt-3">
              <table class="w-full h-46 text-sm text-left rtl:text-right text-gray-500 dark:text-gray-400">
                <thead class="text-xs text-gray-900 uppercase dark:text-gray-400 bg-lime-300 sticky top-0 ">
                  <tr>
                    <th scope="col" class="px-6 py-3 text-center">
                      Device
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Datetime
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Time
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      CO2
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Temp
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Humi
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      EC
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Pressure
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Flowmeters
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Motor
                    </th>
                  </tr>
                </thead>
                <tbody className="">
                  {reversedData1.map((item) => (
                    <tr class="bg-white dark:bg-gray-800" key={item._id}>
                      <td class="px-6 py-4 text-center">{item.device_name}</td>
                      <td class="px-6 py-4 text-center">{item.date}</td>
                      <td class="px-6 py-4 text-center">{item.time}</td>
                      <td class="px-6 py-4 text-center">{item.CO2}</td>
                      <td class="px-6 py-4 text-center">{item.Temp}</td>
                      <td class="px-6 py-4 text-center">{item.Humi}</td>
                      <td class="px-6 py-4 text-center">{item.EC}</td>
                      <td class="px-6 py-4 text-center">{item.Pressure}</td>
                      <td class="px-6 py-4 text-center">{item.Flowmeters}</td>
                      <td class="px-6 py-4 text-center">
                        {item.Motor ? "On" : "Off"}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          {DataList === "user" && (
            <div className="relative h-4/5  overflow-x-auto  rounded-3xl mt-3">
              <table class="w-full text-sm text-left rtl:text-right text-gray-500 dark:text-gray-400">
                <thead class="text-xs text-gray-900 uppercase dark:text-gray-400 bg-lime-300 sticky top-0">
                  <tr>
                    <th scope="col" class="px-6 py-3 text-center">
                      Name
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      User Name
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Phone Number
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Email
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Role
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Login at
                    </th>
                  </tr>
                </thead>
                <tbody>
                  {reversedData1.map((item) => (
                    <tr class="bg-white dark:bg-gray-800" key={item._id}>
                      <td class="px-6 py-4 text-center">{item.full_name}</td>
                      <td class="px-6 py-4 text-center">{item.username}</td>
                      <td class="px-6 py-4 text-center">{item.phone}</td>
                      <td class="px-6 py-4 text-center">{item.email}</td>
                      <td class="px-6 py-4 text-center">{item.role}</td>
                      <td class="px-6 py-4 text-center">{item.lastlogin}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          {DataList === "Control-History" && (
            <div className="relative h-4/5 bg-white overflow-x-auto  rounded-3xl mt-3"></div>
          )}
        </div>
      </div>
    </div>
  );
};

export default DataAnalysis;

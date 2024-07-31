import { React, useEffect, useState } from "react";
import { useNavigate, useLocation } from "react-router-dom";
import { Dialog } from "@headlessui/react";
import axios from "./checkToken";

const Decentralization = () => {
  const [CRfullname, setFullname] = useState("");
  const [CRusername, setUsername] = useState("");
  const [CRpassword, setPassword] = useState("");
  const [CRrole, setRole] = useState("");
  const [CRemail, setEmail] = useState("");
  const [CRphone, setPhone] = useState("");

  const [Display, setDisplay] = useState("1");
  const [data1, setData] = useState([]);
  const [error, setError] = useState("");
  const [errorMail, setErrorMail] = useState("");
  const location = useLocation();
  const navigate = useNavigate();

  const [PassToCheck, SetPassCheck] = useState("");
  const [mode, setMode] = useState([]);
  const [userID, setUserID] = useState([]);
  const [dataChange, setDataChange] = useState("del");
  const [isDialogOpen, setIsDialogOpen] = useState(false);

  const token = localStorage.getItem("token");
  const access_token = "Bearer " + token;

  const { username, role } = location.state || {
    username: localStorage.getItem("username"),
    role: localStorage.getItem("role"),
  };
  const ImgUsr = (usr) => {
    const a = new URL(`/src/assets/user/${usr}.jpg`, import.meta.url).href;
    if (a === "http://34.87.151.244:5173/src/pages/undefined") {
      return new URL(`/src/assets/user/user.jpg`, import.meta.url).href;
    } else {
      return a;
    }
  };
  const validatePhoneNumber = (phone) => {
    if (!/^0\d{9}$/.test(phone)) {
      setError("Invalid phone number");
    } else {
      setError("");
    }
  };

  const handlePhoneChange = (e) => {
    const phone = e.target.value;
    setPhone(phone);
    validatePhoneNumber(phone);
  };

  const validateEmail = (email) => {
    const regex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!regex.test(email)) {
      setErrorMail("Please provide a valid email address.");
    } else {
      setErrorMail("");
    }
  };

  const handleEmailChange = (e) => {
    const email = e.target.value;
    setEmail(email);
    validateEmail(email);
  };

  const openDialog = (mode, idUser) => {
    setMode(mode);
    setUserID(idUser);
    setIsDialogOpen(true);
  };

  const hiddenDialog = async () => {
    setIsDialogOpen(false);
  };
  const closeDialog = async () => {
    try {
      const response = await axios.post(
        `http://34.87.151.244:1506/signup?${registerform}`
      );

      navigate("/user-management");
      loadData();
    } catch (e) {
      console.error("Error logging in:", e);
    } finally {
      setIsDialogOpen(false);
    }
  };
  const ChangeData = async () => {
    const url = `http://34.87.151.244:1506/users/${mode}/${userID}?updated_data=${dataChange}`;

    const dataAdmin = {
      masterusr: localStorage.getItem("username"),
      masterpwd: PassToCheck,
    };

    try {
      const response = await axios.patch(url, dataAdmin, {
        headers: {
          Authorization: access_token,
          accept: "application/json",
          "Content-Type": "application/json",
        },
      });
    } finally {
      loadData();
      setIsDialogOpen(false);
    }
  };
  async function loadData() {
    const response = await axios.get("http://34.87.151.244:1506/history/user", {
      headers: {
        Authorization: access_token,
        accept: "application/json",
        "Content-Type": "application/x-www-form-urlencoded",
      },
    });
    const dt1 = response.data;
    setData(dt1);
  }
  useEffect(() => {
    loadData();
  }, [Display]);
  return (
    <div className="flex  h-full w-full ">
      {/* Dialog*/}
      <Dialog
        open={isDialogOpen}
        onClose={closeDialog}
        className="fixed inset-0 z-10 overflow-y-auto"
      >
        {mode === "create" && (
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
                  onClick={hiddenDialog}
                  className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                >
                  Cancel
                </button>
                <button
                  onClick={closeDialog}
                  className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                >
                  Submit
                </button>
              </div>
            </div>
          </div>
        )}
        {mode === "delete" && (
          <div className="flex items-center bg-opacity-75 bg-black justify-center min-h-screen px-4">
            <div className="relative bg-white rounded-lg max-w-sm mx-auto p-6">
              <div className="text-lg font-bold text-gray-900">Delete User</div>
              <div className="mt-2 text-sm text-gray-500">
                Please enter a password to delete this user.
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
                  onClick={hiddenDialog}
                  className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                >
                  Cancel
                </button>
                <button
                  onClick={ChangeData}
                  className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                >
                  Submit
                </button>
              </div>
            </div>
          </div>
        )}
        {mode === "role" && (
          <div className="flex items-center bg-opacity-75 bg-black justify-center min-h-screen px-4">
            <div className="relative bg-white rounded-lg max-w-sm mx-auto p-6">
              <div className="text-lg font-bold text-gray-900">Change Role</div>
              <div className="mt-2 text-sm text-gray-500">
                Please enter a password to change Role.
              </div>

              <input
                type="password"
                className="mt-4 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:border-green-500 focus:ring focus:ring-green-500 focus:ring-opacity-50"
                onChange={(e) => SetPassCheck(e.target.value)}
              />
              <form className="h-20">
                <label className="block mb-3 p-4">
                  <span className="block text-sm font-bold  text-slate-700">
                    Role
                  </span>
                  <label className="inline-flex items-center w-full text-sm font-semibold ">
                    <select
                      className="text-gray-500 mt-1  px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1select_box"
                      onChange={(e) => setDataChange(e.target.value)}
                    >
                      <option value="Admin">Choice your authority</option>
                      <option value="admin">Admin</option>
                      <option value="user">User</option>
                      <option value="others">Others</option>
                    </select>
                  </label>
                </label>
              </form>
              <div className="mt-4 flex justify-end">
                <button
                  onClick={hiddenDialog}
                  className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                >
                  Cancel
                </button>
                <button
                  onClick={ChangeData}
                  className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                >
                  Submit
                </button>
              </div>
            </div>
          </div>
        )}
        {mode === "password" && (
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
              <div className="mt-2 text-sm text-gray-500">
                Please enter a password to change for {userID}.
              </div>

              <input
                type="password"
                id="password-to-change"
                name="password-to-change"
                className="mt-4 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:border-green-500 focus:ring focus:ring-green-500 focus:ring-opacity-50"
                onChange={(e) => setDataChange(e.target.value)}
              />
              <div className="mt-4 flex justify-end">
                <button
                  onClick={hiddenDialog}
                  className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                >
                  Cancel
                </button>
                <button
                  onClick={ChangeData}
                  className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                >
                  Submit
                </button>
              </div>
            </div>
          </div>
        )}
        {mode === "email" && (
          <div className="flex items-center bg-opacity-75 bg-black justify-center min-h-screen px-4">
            <div className="relative bg-white rounded-lg max-w-sm mx-auto p-6">
              <div className="text-lg font-bold text-gray-900">
                Enter Password to chang Mail
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
              <div className="mt-2 text-sm text-gray-500">
                Please enter an Eamil to change.
              </div>

              <input
                type="email"
                className="mt-4 p-2 block w-full border border-gray-300 rounded-md shadow-sm focus:border-green-500 focus:ring focus:ring-green-500 focus:ring-opacity-50"
                onChange={(e) => setDataChange(e.target.value)}
              />
              <div className="mt-4 flex justify-end">
                <button
                  onClick={hiddenDialog}
                  className="px-4 py-2 bg-red-500 text-white rounded-md hover:bg-red-700 transition-colors duration-150"
                >
                  Cancel
                </button>
                <button
                  onClick={ChangeData}
                  className="ml-2 px-4 py-2 bg-green-500 text-white rounded-md hover:bg-green-700 transition-colors duration-150"
                >
                  Submit
                </button>
              </div>
            </div>
          </div>
        )}
      </Dialog>
      {/* End Dialog*/}
      {/* -------------------------PC View------------------------- */}

      <div className="hidden sm:block w-full h-full   ">
        <div className="h-10 ">
          <div className="w-3/5 search-bar">
            <label className="relative block">
              <span className="sr-only">Search</span>
              <span className="absolute inset-y-0 left-0 flex items-center pl-2">
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
                  className="icon icon-tabler icons-tabler-outline icon-tabler-search"
                >
                  <path stroke="none" d="M0 0h24v24H0z" fill="none" />
                  <path d="M10 10m-7 0a7 7 0 1 0 14 0a7 7 0 1 0 -14 0" />
                  <path d="M21 21l-6 -6" />
                </svg>
              </span>
              <input
                className="placeholder:italic placeholder:text-slate-400 block bg-white w-full border border-slate-300 rounded-md py-2 pl-9 pr-3 shadow-sm focus:outline-none focus:border-sky-500 focus:ring-sky-500 focus:ring-1 sm:text-sm"
                placeholder="Search for anything..."
                type="text"
                name="search"
              />
            </label>
          </div>
        </div>
        <div className="flex h-5/6 mt-4 ">
          <div class="w-2/3 overflow-hidden">
            <div class="relative h-full overflow-y-auto rounded-3xl mt-3">
              <table class="w-full text-sm text-left rtl:text-right text-gray-500 rounded-3xl dark:text-gray-400">
                <thead class="text-xs text-gray-900 uppercase dark:text-gray-400 bg-lime-300 sticky top-0">
                  <tr>
                    <th scope="col" class="px-6 py-3 text-center">
                      Avartar
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Full Name
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      User Name
                    </th>

                    <th scope="col" class="px-6 py-3 text-center">
                      Role
                    </th>
                    <th scope="col" class="px-6 py-3 text-center">
                      Setup
                    </th>
                  </tr>
                </thead>
                <tbody className="rounded-b-2xl">
                  {data1.map((item) => (
                    <tr class="bg-white dark:bg-gray-800" key={item._id}>
                      <td class="px-6 py-4 text-center">
                        <img
                          className="block mx-auto h-16 w-16 rounded-full sm:mx-0 sm:flex-shrink-0"
                          src={ImgUsr(item.username)}
                        ></img>
                      </td>
                      <td class="px-6 py-4 text-center">{item.full_name}</td>
                      <td class="px-6 py-4 text-center">{item.username}</td>
                      <td class="px-6 py-4 text-center">{item.role}</td>
                      <td class="px-6 py-4 text-center">
                        <label className="inline-flex items-center text-sm font-semibold overflow-auto ">
                          <select
                            className="text-gray-500 mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1select_box"
                            name="data-list"
                            id="data-list"
                            onChange={(e) =>
                              openDialog(e.target.value, item.username)
                            }
                          >
                            <option elemet="admin">Edit </option>
                            <option value="password">Change Password</option>
                            <option value="role">Change Role</option>
                            <option value="mail">Change Email</option>
                            <option value="delete">Delete User</option>
                          </select>
                        </label>
                      </td>
                    </tr>
                  ))}
                  {/* {data1.map((item) => (
                    <tr className="bg-white dark:bg-gray-800" key={item._id}>
                      <td className="px-6 py-4 text-center">
                        <img
                          className="block mx-auto h-16 w-16 rounded-full sm:mx-0 sm:flex-shrink-0"
                          src={ImgUsr(item.username)}
                          alt={`Profile of ${item.full_name}`}
                        />
                      </td>
                      <td className="px-6 py-4 text-center">
                        {item.full_name}
                      </td>
                      <td className="px-6 py-4 text-center">{item.username}</td>
                      <td className="px-6 py-4 text-center">{item.role}</td>
                      <td className="px-6 py-4 text-center">
                        <label className="inline-flex items-center text-sm font-semibold overflow-auto">
                          <select
                            className="text-gray-500 mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                            name="data-list"
                            id="data-list"
                            onChange={(e) =>
                              openDialog(e.target.value, item.username)
                            }
                          >
                            <option value="admin">Edit</option>
                            <option value="password">Change Password</option>
                            <option value="role">Change Role</option>
                            <option value="mail">Change Email</option>
                            <option value="delete">Delete User</option>
                          </select>
                        </label>
                      </td>
                    </tr>
                  ))} */}
                </tbody>
              </table>
            </div>
          </div>
          <div class="w-1/3 ml-3">
            <div className="p-4  flex flex-col h-full rounded-3xl  bg-white">
              <p className="ml-3 font-bold text-center">CREATE ACCOUNT</p>
              <form className="h-20">
                <label className="block mb-3 p-4 ">
                  <span className="block text-sm font-bold text-slate-700">
                    Full Name
                  </span>
                  <input
                    type="text"
                    name="fullname"
                    className=" mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                    placeholder="First and last name"
                    value={CRfullname}
                    onChange={(e) => setFullname(e.target.value)}
                  />
                </label>
              </form>

              <form className="h-20">
                <label className="block mb-3 p-4">
                  <span className="block text-sm font-bold text-slate-700">
                    Username
                  </span>
                  <input
                    type="text"
                    name="username"
                    className=" mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                    placeholder="The name used to log in"
                    value={CRusername}
                    onChange={(e) => setUsername(e.target.value)}
                  />
                </label>
              </form>

              <form className="h-20">
                <label className="block mb-3 p-4">
                  <span className="block text-sm font-bold text-slate-700">
                    Password
                  </span>
                  <input
                    type="password"
                    name="new-password"
                    className=" mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                    placeholder="Enter your passwork"
                    value={CRpassword}
                    onChange={(e) => setPassword(e.target.value)}
                  />
                </label>
              </form>

              <form className="h-20">
                <label className="block mb-3 p-4">
                  <span className="block text-sm font-bold  text-slate-700">
                    Role
                  </span>
                  <label className="inline-flex items-center w-full text-sm font-semibold ">
                    <select
                      className="text-gray-500 mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1select_box"
                      onChange={(e) => setRole(e.target.value)}
                    >
                      <option value="Admin">Choice your authority</option>
                      <option value="admin">Admin</option>
                      <option value="user">User</option>
                      <option value="others">Others</option>
                    </select>
                  </label>
                </label>
              </form>

              <form className="h-20">
                <label className="block mb-3 p-4">
                  <span className="block text-sm font-bold text-slate-700">
                    Email
                  </span>
                  <div>
                    <input
                      type="email"
                      name="username"
                      className={` mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none ${
                        errorMail ? "border-red-500" : "focus:border-sky-500"
                      } ${
                        errorMail ? "focus:ring-red-500" : "focus:ring-sky-500"
                      } block w-full rounded-md sm:text-sm focus:ring-1`}
                      placeholder="abc@gmail.com"
                      value={CRemail}
                      onChange={handleEmailChange}
                      required
                    />
                    {errorMail && (
                      <p className="mt-2 text-pink-600 text-xs italic">
                        {errorMail}
                      </p>
                    )}
                  </div>
                </label>
              </form>
              {errorMail ? <div className="mt-2"></div> : ""}
              <form className="h-20">
                <label className="block mb-3 p-4">
                  <span className="block text-sm font-bold text-slate-700">
                    Phone Number
                  </span>
                  <div>
                    <input
                      type="number"
                      name="PhoneNumber"
                      className={` mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none ${
                        error ? "border-red-500" : "focus:border-sky-500"
                      } ${
                        error ? "focus:ring-red-500" : "focus:ring-sky-500"
                      } block w-full rounded-md sm:text-sm focus:ring-1`}
                      placeholder="0xxxxxxxxx"
                      value={CRphone}
                      onChange={handlePhoneChange}
                      // onBlur={handleBlur}
                      required
                    />
                    {error && (
                      <p className="mt-2 text-pink-600 text-xs italic">
                        {error}
                      </p>
                    )}
                  </div>
                </label>
              </form>
              <div className="flex justify-center mt-6">
                <button
                  value="create"
                  onClick={(e) => openDialog(e.target.value, null)}
                  className="px-4 py-2 button-create-user bg-green-500 transition-colors duration-150 cursor-pointer hover:bg-green-900 text-cyan-50 rounded-full font-bold"
                >
                  Create
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>

      {/* -----------------------MObile View----------------------- */}

      <div className=" sm:hidden h-full w-screen ">
        <ul className=" p-2 flex justify-center w-full overflow-x-auto ">
          <li class="p-4   list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded ">
            <span
              class="ml-2 underline hover:underline-offset-8  "
              onClick={() => {
                setDisplay("1");
              }}
            >
              Create Account
            </span>
          </li>
          <li class="list-none flex items-center text-green-500 font-bold cursor-pointer  hover:text-yellow-500 rounded p-4">
            <span
              class="ml-2 underline hover:underline-offset-8 "
              onClick={() => {
                setDisplay("0");
              }}
            >
              Management
            </span>
          </li>
        </ul>
        {Display === "1" && (
          <div className="p-4  bg-white bg-opacity-75 flex flex-col h-fit w-full rounded-3xl gap-0.75 ">
            <form className="h-20">
              <label className="block mb-3 p-4 ">
                <span className="block text-sm font-bold text-slate-700">
                  Full Name
                </span>
                <input
                  type="text"
                  name="fullname"
                  className=" mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                  placeholder="Thangwf user tên gì :)"
                  value={CRfullname}
                  onChange={(e) => setFullname(e.target.value)}
                />
              </label>
            </form>

            <form className="h-20">
              <label className="block mb-3 p-4">
                <span className="block text-sm font-bold text-slate-700">
                  Username
                </span>
                <input
                  type="text"
                  name="username"
                  className=" mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                  placeholder="cho no name deex nho"
                  value={CRusername}
                  onChange={(e) => setUsername(e.target.value)}
                />
              </label>
            </form>

            <form className="h-20">
              <label className="block mb-3 p-4">
                <span className="block text-sm font-bold text-slate-700">
                  Password
                </span>
                <input
                  type="password"
                  name="new-password"
                  className=" mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1"
                  placeholder="Đặt cái mật khẩu nhiều kí tự vào :)"
                  value={CRpassword}
                  onChange={(e) => setPassword(e.target.value)}
                />
              </label>
            </form>

            <form className="h-20">
              <label className="block mb-3 p-4">
                <span className="block text-sm font-bold  text-slate-700">
                  Role
                </span>
                <label className="inline-flex items-center w-full text-sm font-semibold ">
                  <select
                    className="text-gray-500 mt-1 h-12 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none focus:border-sky-500 focus:ring-sky-500 block w-full rounded-md sm:text-sm focus:ring-1select_box"
                    onChange={(e) => setRole(e.target.value)}
                  >
                    <option value="Admin">Choice your authority</option>
                    <option value="admin">Admin</option>
                    <option value="user">User</option>
                    <option value="others">Others</option>
                  </select>
                </label>
              </label>
            </form>

            <form className="h-20">
              <label className="block mb-3 p-4">
                <span className="block text-sm font-bold text-slate-700">
                  Email
                </span>
                <div>
                  <input
                    type="email"
                    name="username"
                    className={` mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none ${
                      errorMail ? "border-red-500" : "focus:border-sky-500"
                    } ${
                      errorMail ? "focus:ring-red-500" : "focus:ring-sky-500"
                    } block w-full rounded-md sm:text-sm focus:ring-1`}
                    placeholder="Nhập cái email cho chuẩn vào :)"
                    value={CRemail}
                    onChange={handleEmailChange}
                    required
                  />
                  {errorMail && (
                    <p className="mt-2 text-pink-600 text-xs italic">
                      {errorMail}
                    </p>
                  )}
                </div>
              </label>
            </form>
            {errorMail ? <br></br> : ""}
            <form className="h-20">
              <label className="block mb-3 p-4">
                <span className="block text-sm font-bold text-slate-700">
                  Phone Number
                </span>
                <div>
                  <input
                    type="number"
                    name="PhoneNumber"
                    className={` mt-1 px-3 py-2 bg-white border shadow-sm border-slate-300 placeholder-slate-300 focus:outline-none ${
                      error ? "border-red-500" : "focus:border-sky-500"
                    } ${
                      error ? "focus:ring-red-500" : "focus:ring-sky-500"
                    } block w-full rounded-md sm:text-sm focus:ring-1`}
                    placeholder="Biết ghi số điện thoại khum?"
                    value={CRphone}
                    onChange={handlePhoneChange}
                    // onBlur={handleBlur}
                    required
                  />
                  {error && (
                    <p className="mt-2 text-pink-600 text-xs italic">{error}</p>
                  )}
                </div>
              </label>
            </form>
            <div className="flex justify-center mt-8">
              <button
                value="create"
                onClick={(e) => openDialog(e.target.value, null)}
                className="px-4 py-2 button-create-user bg-green-500 transition-colors duration-150 cursor-pointer hover:bg-green-900 text-cyan-50 rounded-full font-bold"
              >
                Create
              </button>
            </div>
          </div>
        )}
        {Display === "0" && (
          <div class=" w-full overflow-hidden">
            <div class="relative h-full overflow-y-auto  mt-3">
              <table class="w-full text-sm text-left rtl:text-right text-gray-500  dark:text-gray-400">
                <thead class="text-xs text-gray-900 uppercase dark:text-gray-400 bg-lime-300 sticky top-0">
                  <tr>
                    <th scope="col" class="px-2 py-2 text-center">
                      Avartar
                    </th>
                    <th scope="col" class="px-2 py-2 text-center">
                      Full Name
                    </th>
                    <th scope="col" class="px-2 py-2 text-center">
                      User Name
                    </th>
                    <th scope="col" class="px-2 py-2 text-center">
                      Role
                    </th>
                    <th scope="col" class="px-2 py-2 text-center">
                      Setup
                    </th>
                  </tr>
                </thead>
                <tbody class="rounded-b-2xl">
                  {data1.map((item) => (
                    <tr class="bg-white dark:bg-gray-800" key={item._id}>
                      <td class="px-2 py-2 text-center">
                        <img
                          class="block mx-auto h-16 w-16 rounded-full sm:mx-0 sm:flex-shrink-0"
                          src={ImgUsr(item.username)}
                        />
                      </td>
                      <td class="px-2 py-2 text-center">{item.full_name}</td>
                      <td class="px-2 py-2 text-center sm:hidden md:table-cell">
                        {item.username}
                      </td>
                      <td class="px-2 py-2 text-center">{item.role}</td>
                      <td class="px-2 py-2 text-center">
                        <label class="inline-flex w-6 items-center text-sm font-semibold  ">
                          <select
                            class="w-6"
                            name="data-list"
                            id="data-list"
                            onChange={(e) =>
                              openDialog(e.target.value, item.username)
                            }
                          >
                            <option value="null"></option>
                            <option value="password">Change Password</option>
                            <option value="role">Change Role</option>
                            <option value="email">Change Email</option>
                            <option value="delete">Delete User</option>
                          </select>
                        </label>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default Decentralization;

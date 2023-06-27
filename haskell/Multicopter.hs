{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Multicopter (runMulticopter) where

import Control.Applicative
import Network.Socket
import Network.Socket.ByteString -- from network
import Data.ByteString.Internal
import Data.Either.Utils -- from MissingH
import Data.Serialize -- from cereal

import Types

-- Adapted from http://book.realworldhaskell.org/read/sockets-and-syslog.html

makeUdpSocket :: String -> IO (Socket, SockAddr)
makeUdpSocket port =
    do 
       addrInfo <- getAddrInfo (Just (defaultHints {addrFlags = [AI_PASSIVE]})) Nothing (Just port)
       let addr = head addrInfo
       sock <- socket (addrFamily addr) Datagram defaultProtocol
       return (sock, (addrAddress addr))


runMulticopter :: PidController -> Mixer -> IO ()
runMulticopter controller mixer = withSocketsDo $

    do 

       (telemetryServerSocket, telemetryServerSocketAddress) <- makeUdpSocket "5001"

       (motorClientSocket, motorClientSocketAddress) <- makeUdpSocket "5000"

       bind telemetryServerSocket telemetryServerSocketAddress

       putStrLn "Hit the Play button ..."

       processMessages telemetryServerSocket motorClientSocket motorClientSocketAddress (PidControllerState 0 0)

    where processMessages telemetryServerSocket motorClientSocket motorClientSockAddr controllerState =
              do 

                  (msgIn, _) <- Network.Socket.ByteString.recvFrom telemetryServerSocket 160

                  let doubles = bytesToDoubles msgIn

                  let time = head doubles
                  let vehicleState = makeState (slice 1 13 doubles)

                  let (demands, newControllerState) = controller time vehicleState  demands controllerState
                  let motors = mixer demands
                  _ <- Network.Socket.ByteString.sendTo
                        motorClientSocket
                        (doublesToBytes [(m1 motors), (m2 motors), (m3 motors), (m4 motors)])
                        motorClientSockAddr

                  if time >= 0
                  then processMessages telemetryServerSocket motorClientSocket motorClientSockAddr newControllerState
                  else putStrLn "Done"
                      

-- https://stackoverflow.com/questions/20912582/haskell-bytestring-to-float-array

doublesToBytes :: [Double] -> ByteString
doublesToBytes = runPut . mapM_ putFloat64le

bytesToDoubles :: ByteString -> [Double]
bytesToDoubles bs = (fromRight ((runGet $ many getFloat64le) bs))

-- https://stackoverflow.com/questions/4597820/does-haskell-have-list-slices-i-e-python
slice :: Int -> Int -> [a] -> [a]
slice from to xs = take (to - from + 1) (drop from xs)

% Publish the next taskIndex_.
function sendNextTask(taskIndex)
    global Int16Msg;
    global PGEllaTaskIndexPub;

    Int16Msg.setData(taskIndex);
    PGEllaTaskIndexPub.publish(Int16Msg);
end
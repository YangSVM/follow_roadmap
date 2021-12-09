# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
"""Client and server classes corresponding to protobuf-defined services."""
import grpc

from proto import adm2ctrl_pb2 as proto_dot_adm2ctrl__pb2


class DtoCStub(object):
    """decesion to control service
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.ExecZhncha = channel.unary_unary(
                '/adm2ctrl.DtoC/ExecZhncha',
                request_serializer=proto_dot_adm2ctrl__pb2.ZhnchaRequest.SerializeToString,
                response_deserializer=proto_dot_adm2ctrl__pb2.ZhnchaReply.FromString,
                )
        self.ExecTasks = channel.unary_unary(
                '/adm2ctrl.DtoC/ExecTasks',
                request_serializer=proto_dot_adm2ctrl__pb2.TasksRequest.SerializeToString,
                response_deserializer=proto_dot_adm2ctrl__pb2.TasksReply.FromString,
                )
        self.ExecAttack = channel.unary_unary(
                '/adm2ctrl.DtoC/ExecAttack',
                request_serializer=proto_dot_adm2ctrl__pb2.AttackRequest.SerializeToString,
                response_deserializer=proto_dot_adm2ctrl__pb2.AttackReply.FromString,
                )


class DtoCServicer(object):
    """decesion to control service
    """

    def ExecZhncha(self, request, context):
        """send zhncha command
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ExecTasks(self, request, context):
        """send task info
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def ExecAttack(self, request, context):
        """send attack pairs
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_DtoCServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'ExecZhncha': grpc.unary_unary_rpc_method_handler(
                    servicer.ExecZhncha,
                    request_deserializer=proto_dot_adm2ctrl__pb2.ZhnchaRequest.FromString,
                    response_serializer=proto_dot_adm2ctrl__pb2.ZhnchaReply.SerializeToString,
            ),
            'ExecTasks': grpc.unary_unary_rpc_method_handler(
                    servicer.ExecTasks,
                    request_deserializer=proto_dot_adm2ctrl__pb2.TasksRequest.FromString,
                    response_serializer=proto_dot_adm2ctrl__pb2.TasksReply.SerializeToString,
            ),
            'ExecAttack': grpc.unary_unary_rpc_method_handler(
                    servicer.ExecAttack,
                    request_deserializer=proto_dot_adm2ctrl__pb2.AttackRequest.FromString,
                    response_serializer=proto_dot_adm2ctrl__pb2.AttackReply.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'adm2ctrl.DtoC', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class DtoC(object):
    """decesion to control service
    """

    @staticmethod
    def ExecZhncha(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/adm2ctrl.DtoC/ExecZhncha',
            proto_dot_adm2ctrl__pb2.ZhnchaRequest.SerializeToString,
            proto_dot_adm2ctrl__pb2.ZhnchaReply.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def ExecTasks(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/adm2ctrl.DtoC/ExecTasks',
            proto_dot_adm2ctrl__pb2.TasksRequest.SerializeToString,
            proto_dot_adm2ctrl__pb2.TasksReply.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def ExecAttack(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/adm2ctrl.DtoC/ExecAttack',
            proto_dot_adm2ctrl__pb2.AttackRequest.SerializeToString,
            proto_dot_adm2ctrl__pb2.AttackReply.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)


class CtoDStub(object):
    """control to decision service
    """

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.FbExecInfo = channel.unary_unary(
                '/adm2ctrl.CtoD/FbExecInfo',
                request_serializer=proto_dot_adm2ctrl__pb2.ExecInfoRequest.SerializeToString,
                response_deserializer=proto_dot_adm2ctrl__pb2.ExecInfoReply.FromString,
                )


class CtoDServicer(object):
    """control to decision service
    """

    def FbExecInfo(self, request, context):
        """send task execution feedback
        """
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_CtoDServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'FbExecInfo': grpc.unary_unary_rpc_method_handler(
                    servicer.FbExecInfo,
                    request_deserializer=proto_dot_adm2ctrl__pb2.ExecInfoRequest.FromString,
                    response_serializer=proto_dot_adm2ctrl__pb2.ExecInfoReply.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'adm2ctrl.CtoD', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class CtoD(object):
    """control to decision service
    """

    @staticmethod
    def FbExecInfo(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            insecure=False,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/adm2ctrl.CtoD/FbExecInfo',
            proto_dot_adm2ctrl__pb2.ExecInfoRequest.SerializeToString,
            proto_dot_adm2ctrl__pb2.ExecInfoReply.FromString,
            options, channel_credentials,
            insecure, call_credentials, compression, wait_for_ready, timeout, metadata)

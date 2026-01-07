package com.soonerrobotics;

import java.util.Objects;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

public class Result<S> implements Supplier<S> {

    /**
     * Creates a successful Result containing the given value.
     */
    public static <S> Result<S> success(S value) {
        return new Result<>(null, Objects.requireNonNull(value));
    }

    /**
     * Creates a failed Result containing the given error.
     */
    public static <S> Result<S> fail(Exception error) {
        return new Result<>(Objects.requireNonNull(error), null);
    }

    /**
     * Creates a failed Result containing an error with the given message.
     */
    public static <S> Result<S> fail(String errorMessage) {
        return new Result<>(new Exception(errorMessage), null);
    }

    private final Exception _error;
    private final S _value;

    private Result(Exception error, S value) {
        this._error = error;
        this._value = value;
    }

    /**
     * Returns true if this Result represents an error.
     * @return true if this Result represents an error.
     */
    public boolean isError() {
        return _error != null;
    }

    /**
     * Returns true if this Result represents a success.
     * @return true if this Result represents a success.
     */
    public boolean isSuccess() {
        return _error == null;
    }

    /**
     * Returns the error contained in this Result.
     * @return the error contained in this Result.
     * @throws IllegalStateException if this Result does not represent an error.
     */    
    public Exception getError() throws IllegalStateException {
        if (!isError()) {
            throw new IllegalStateException("No error present");
        }

        return _error;
    }

    /**
     * Returns the value contained in this Result.
     * @return the value contained in this Result.
     * @throws IllegalStateException if this Result does not represent a success.
     */
    @Override
    public S get() throws IllegalStateException {
        if (isError()) {
            throw new IllegalStateException("No value present");
        }

        return _value;
    }

    /**
     * Returns the value contained in this Result, or the given other value if this Result represents an error.
     * @param other the other value to return if this Result represents an error.
     * @return the value contained in this Result, or the given other value if this Result represents an error.
     */
    public S getOrElse(S other) {
        return isSuccess() ? _value : other;
    }

    /**
     * Returns the value contained in this Result, or the result of applying the given function to the error if this Result represents an error.
     * @param otherFunc the function to apply to the error if this Result represents an error.
     * @return the value contained in this Result, or the result of applying the given function to the error if this Result represents an error.
     */
    public S getOrElse(Function<Exception, S> otherFunc) {
        return isSuccess() ? _value : otherFunc.apply(_error);
    }

    /**
     * Returns the value contained in this Result, or throws the error if this Result represents an error.
     * @return the value contained in this Result.
     * @throws Exception if this Result represents an error.
     */
    public S getOrThrow() throws Exception {
        if (isError()) {
            throw _error;
        }

        return _value;
    }

    /**
     * Maps the value contained in this Result using the given function, if this Result represents a success.
     * @param <N> the type of the value contained in the new Result.
     * @param mapper the function to apply to the value if this Result represents a success.
     * @return a new Result containing the mapped value, or the same error if this Result represents an error.
     */
    public <N> Result<N> map(Function<S, N> mapper) {
        if (isError()) {
            return Result.fail(_error);
        }

        return Result.success(mapper.apply(_value));
    }

    /**
     * Maps the error contained in this Result using the given function, if this Result represents an error.
     * @param errorMapper the function to apply to the error if this Result represents an error.
     * @return a new Result containing the same value if this Result represents a success, or the mapped error if this Result represents an error.
     */
    public Result<S> mapError(Function<Exception, Exception> errorMapper) {
        if (isSuccess()) {
            return this;
        }

        return Result.fail(errorMapper.apply(_error));
    }

    /**
     * If this Result represents an error, applies the given consumer to the error.
     * @param errorConsumer the consumer to apply to the error if this Result represents an error.
     * @return this Result.
     */
    public Result<S> ifError(Consumer<Exception> errorConsumer) {
        if (isError()) {
            errorConsumer.accept(_error);
        }

        return this;
    }

    /**
     * If this Result represents a success, applies the given consumer to the value.
     * @param valueConsumer the consumer to apply to the value if this Result represents a success.
     * @return this Result.
     */
    public Result<S> ifSuccess(Consumer<S> valueConsumer) {
        if (isSuccess()) {
            valueConsumer.accept(_value);
        }

        return this;
    }

    /**
     * Wraps the error contained in this Result using the given function and message, if this Result represents an error.
     * Example: wrapError((msg, err) -> new RuntimeException(msg, err), "Failed to do something")
     * @param f the function to apply to the error and message if this Result represents an error.
     * @param message the message to pass to the function.
     * @return a new Result containing the same value if this Result represents a success, or the wrapped error if this Result represents an error.
     */
    public Result<S> wrapError(BiFunction<String, Exception, Exception> f, String message) {
        if (isSuccess()) {
            return this;
        }

        return Result.fail(f.apply(message, _error));
    }

    /**
     * Wraps the error contained in this Result using the given function, if this Result represents an error.
     * Example: wrapError(err -> new RuntimeException("Failed to do something", err))
     * @param f the function to apply to the error if this Result represents an error.
     * @return a new Result containing the same value if this Result represents a success, or the wrapped error if this Result represents an error.
     */
    public Result<S> wrapError(Function<Exception, Exception> f) {
        return mapError(f);
    }

    /**
     * Converts this Result to an Optional.
     * @return an Optional containing the value if this Result represents a success, or an empty Optional if this Result represents an error.
     */
    public Optional<S> asOptional() {
        return isSuccess() ? Optional.of(_value) : Optional.empty();
    }
}
